import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
const lightMuiTheme = getMuiTheme();
const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
import RaisedButton from 'material-ui/RaisedButton';
import DropDownMenu from 'material-ui/DropDownMenu';
import MenuItem from 'material-ui/MenuItem';


type Props = {
    openPorts: string[],
    connectToAircraftAjax: () => void,
    sendWPsToACAjax: () => void,
    handleSelectedAircraftPortChange: (port: number) => void
}

type State = {
    selectedAircraftPort?: number
}

export class AircraftConnectionButtons extends React.Component<Props, State> {

    constructor() {
        super();

        this.state = {
            selectedAircraftPort: 0
        }
    }

    handleDropdownChange = (event: any, index: number, value: number) => {
        this.setState({selectedAircraftPort: value});
        this.props.handleSelectedAircraftPortChange(value);
    }


    render() {
        const backgroundColors = ['rgba(255,0,0,0.2)', 'rgba(0,0,255,0.2)', 'rgba(0,0,0,0.2)', 'rgba(0,255,0,0.2)', 'rgba(255,255,0,0.2)', 'rgba(255,153,0,0.2)'];
        const aircraftCommsContainer = { position: 'absolute', bottom: 20, left: 15, zIndex: 999};
        const buttonStyle = { marginLeft: 10 };


        let connectOrDisconnectButton: any = null;
        connectOrDisconnectButton = (<MuiThemeProvider muiTheme={lightMuiTheme}>
            <RaisedButton backgroundColor={backgroundColors[this.state.selectedAircraftPort]} style={buttonStyle} label="Connect to aircraft" onClick={this.props.connectToAircraftAjax}/>
        </MuiThemeProvider>);

        return(
            <div>
                {this.props.openPorts.length > 0 &&
                    <div style={aircraftCommsContainer}>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <DropDownMenu style={{width: 150, backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedAircraftPort} onChange={this.handleDropdownChange}>
                            {this.props.openPorts.map((item: string, i: number) => {
                                return(
                                    <MenuItem key={i} value={i} primaryText={this.props.openPorts[i]} label={this.props.openPorts[i]} />
                                );
                            })}
                            </DropDownMenu>
                        </MuiThemeProvider>

                        {connectOrDisconnectButton}

                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton backgroundColor={backgroundColors[this.state.selectedAircraftPort]} style={buttonStyle} label="Send waypoints to aircraft" onClick={this.props.sendWPsToACAjax}/>
                        </MuiThemeProvider>
                    </div>
                }
            </div>
        )
    }
}

