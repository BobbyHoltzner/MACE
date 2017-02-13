import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';

import RaisedButton from 'material-ui/RaisedButton';
import DropDownMenu from 'material-ui/DropDownMenu';
import MenuItem from 'material-ui/MenuItem';
import * as colors from 'material-ui/styles/colors';


type Props = {
    connectedVehicles: VehicleMapType,
    onSelectedAircraftChange: (id: string) => void,
    onAircraftCommand: (id: string, command: string) => void
}

type State = {
    selectedAircraftID?: string
}

export class VehicleCommandsContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            selectedAircraftID: "0"
        }
    }

    handleDropdownChange = (event: any, index: number, value: string) => {
        this.setState({selectedAircraftID: value});
        this.props.onSelectedAircraftChange(value);
    }


    render() {

        const aircraftCommsContainer = {
            // backgroundColor: colors.orange700,
            position: 'absolute',
            bottom: 0,
            left: 0,
            height: 64,
            zIndex: 9999,
            // width: 100 + "%",
            display: "flex",
            justifyContent: "center",
            alignItems: "center"
        };
        const buttonStyle = { height: 44 };


        let vehicleIDs: JSX.Element[] = [];
        for( let key in this.props.connectedVehicles ){
            // let vehicle = this.props.connectedVehicles[key];

            vehicleIDs.push(
                <MenuItem key={key} value={key} primaryText={key} label={key} />
            );
        }

        return(
            <div>
                {Object.keys(this.props.connectedVehicles).length > 0 &&
                    <div style={aircraftCommsContainer}>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <DropDownMenu style={{width: 150, backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedAircraftID} onChange={this.handleDropdownChange}>
                                <MenuItem value={"0"} primaryText={"1"} label={"id 1"} />
                                <MenuItem value={"1"} primaryText={"2"} label={"id 2"} />
                                {vehicleIDs}
                            </DropDownMenu>
                        </MuiThemeProvider>

                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton style={buttonStyle} label="Launch" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "Launch")}/>
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton style={buttonStyle} label="Land" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "Land")}/>
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton style={buttonStyle} label="RTL" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "RTL")}/>
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                            <RaisedButton style={buttonStyle} label="Send WPs" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SendWPs")}/>
                        </MuiThemeProvider>
                    </div>
                }
            </div>
        )

    }
}
