import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
const lightMuiTheme = getMuiTheme();
const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
import {CardTitle, CardText} from 'material-ui/Card';
import Checkbox from 'material-ui/Checkbox';
import RaisedButton from 'material-ui/RaisedButton';

type Props = {
    sendAircraftCommand: (command: string) => void
}

type State = {
    sendToAllAircraft?: boolean,
    useTestPoints?: boolean
}

export class AircraftCommandsControl extends React.Component<Props, State> {

    constructor() {
        super();

        this.state = {
            sendToAllAircraft: false,
            useTestPoints: false
        }
    }


    render() {

        const aircraftCommand = { width: 100 + '%', marginBottom: 16 };
        const checkbox = { marginBottom: 16 }


        return(
            <div>
                <CardTitle title="Aircraft commands" />
                <CardText>

                    <Checkbox
                        label="Send to all aircraft"
                        style={checkbox}
                        onCheck={() => this.setState({sendToAllAircraft: !this.state.sendToAllAircraft})}
                        />

                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Launch" onClick={() => this.props.sendAircraftCommand('launch')}/>
                    </MuiThemeProvider>
                    {/*
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Loiter" onClick={() => this.aircraftCommand('loiter')}/>
                    </MuiThemeProvider>
                    */}
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Return to Launch" onClick={() => this.props.sendAircraftCommand('rtl')}/>
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Land" onClick={() => this.props.sendAircraftCommand('land')}/>
                    </MuiThemeProvider>
                    {/*
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                    <RaisedButton style={aircraftCommand} label="Start Mission" onClick={() => this.aircraftCommand('mission')}/>
                    </MuiThemeProvider>
                    */}
                    {!this.state.useTestPoints &&
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Switch to Guided" onClick={() => this.props.sendAircraftCommand('guided')}/>
                    </MuiThemeProvider>
                    }

                </CardText>
            </div>
        )
    }
}

