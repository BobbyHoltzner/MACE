import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
const lightMuiTheme = getMuiTheme();
const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
import Toggle from 'material-ui/Toggle';
import {Card} from 'material-ui/Card';

import {AircraftCommandsControl} from './AircraftCommandsControl';
import {PathGenerationControl} from './PathGenerationControl';

type Props = {
    backgroundColor: any,
    selectedAircraftPort: number,
    sendAircraftCommand: (command: string) => void,
    updatePathDirection: (path: string) => void,
    handleSliderChange: (e: any, value: number, slider: string) => void,
    generateWaypointsAjax: () => void,
    writeToFile: () => void
}

type State = {
    showAircraftCommands?: boolean
}

export class ControlContainer extends React.Component<Props, State> {

    constructor() {
        super();

        this.state = {
            showAircraftCommands: false
        }
    }


    render() {

        const backgroundColors = ['rgba(255,0,0,0.2)', 'rgba(0,0,255,0.2)', 'rgba(0,0,0,0.2)', 'rgba(0,255,0,0.2)', 'rgba(255,255,0,0.2)', 'rgba(255,153,0,0.2)'];
        const sliderContainer = { position: 'absolute', bottom: 15, right: 15, zIndex: 999, width: 500};        
        const toggleStyle = { marginBottom: 16 };
        const toggleContainer = { position: 'relative', width: 200, top: 16, right: -275 };

        return(
            <div style={sliderContainer}>
                <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card containerStyle={{backgroundColor: backgroundColors[this.props.selectedAircraftPort]}}>
                    <div style={toggleContainer}>
                        <Toggle
                            label="Show Path/Show Aircraft Commands"
                            style={toggleStyle}
                            onToggle={() => this.setState({showAircraftCommands: !this.state.showAircraftCommands})}
                            />
                    </div>

                    {!this.state.showAircraftCommands &&
                        <PathGenerationControl                        
                            updatePathDirection={(path: string) => this.props.updatePathDirection(path)}
                            handleSliderChange={(e: any, value: number, slider: string) => this.props.handleSliderChange(e, value, slider)}
                            generateWaypointsAjax={this.props.generateWaypointsAjax}
                            writeToFile={this.props.writeToFile}
                         />                    
                    }
                    {this.state.showAircraftCommands &&
                        <AircraftCommandsControl
                            sendAircraftCommand={(command: string) => this.props.sendAircraftCommand(command)}
                         />              
                    }
                </Card>
                </MuiThemeProvider>
            </div>
        )
    }
}

