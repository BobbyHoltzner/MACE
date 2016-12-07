import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
const lightMuiTheme = getMuiTheme();
const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
import {CardTitle, CardText} from 'material-ui/Card';
import {RadioButton, RadioButtonGroup} from 'material-ui/RadioButton';
import Slider from 'material-ui/Slider';
import RaisedButton from 'material-ui/RaisedButton';

type Props = {
    updatePathDirection: (path: string) => void,
    handleSliderChange: (e: any, value: number, slider: string) => void,
    generateWaypointsAjax: () => void,
    writeToFile: () => void
}

type State = {
    pSliderVal?: number,
    gridSliderVal?: number,
    disableRegenerate?: boolean,
    pathDirection?: string,
    disableWriteToFile?: boolean 
}

export class PathGenerationControl extends React.Component<Props, State> {

    constructor() {
        super();

        this.state = {
            pSliderVal: 0.05,
            gridSliderVal: 1.5,
            disableRegenerate: true,
            pathDirection: 'EastWest',
            disableWriteToFile: false
        }
    }


    render() {

        const radioContainer = {marginBottom: 16};
        const radioButton = {display: 'inline-block', width: '150px'};
        const centerButtonStyle = { width: 100 + '%' };

        return(
            <div>
                <CardTitle title="Adjust path parameters" />
                <CardText>
                    <div style={{marginBottom: 8}}>
                        Path direction:
                    </div>
                    <RadioButtonGroup style={radioContainer} name="pathDirection" defaultSelected="EastWest" onChange={(e: any, selected: string) => this.props.updatePathDirection(selected)}>
                        <RadioButton
                            value="EastWest"
                            label="East-West"
                            style={radioButton}
                        />
                        <RadioButton
                            value="NorthSouth"
                            label="North-South"
                            style={radioButton}
                        />
                    </RadioButtonGroup>

                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <Slider
                            description={'Current value for "p": ' + this.state.pSliderVal}
                            min={0.01}
                            max={1}
                            value={this.state.pSliderVal}
                            onChange={(e: any, value: number) => this.props.handleSliderChange(e, value, 'pVal')}
                            />
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <Slider
                            description={'Current value for grid density: ' + this.state.gridSliderVal}
                            min={0.25}
                            max={2}
                            value={this.state.gridSliderVal}
                            onChange={(e: any, value: number) => this.props.handleSliderChange(e, value, 'grid')}
                            />
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={centerButtonStyle} disabled={this.state.disableRegenerate} label="Regenerate Waypoints" onClick={this.props.generateWaypointsAjax}/>
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={centerButtonStyle} disabled={this.state.disableWriteToFile} label="Write Waypoints to file(s)" onClick={this.props.writeToFile}/>
                    </MuiThemeProvider>
                </CardText>
            </div>
        )
    }
}

