import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';

import RaisedButton from 'material-ui/RaisedButton';
import DropDownMenu from 'material-ui/DropDownMenu';
import MenuItem from 'material-ui/MenuItem';
// import * as colors from 'material-ui/styles/colors';
// import FontIcon from 'material-ui/FontIcon';
// import { Grid, Col, Row } from 'react-bootstrap';

import { Vehicle } from '../Vehicle';


type Props = {
    connectedVehicles: {[id: string]: Vehicle},
    selectedAircraftID: string,
    onSelectedAircraftChange: (id: string) => void,
    onAircraftCommand: (id: string, tcpCommand: string, vehicleCommand: string) => void,
    handleTakeoff: () => void
}

type State = {
    selectedAircraftID?: string,
    vehicleArmed?: boolean
}

export class VehicleCommandsContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            selectedAircraftID: this.props.selectedAircraftID,
            vehicleArmed: false
        }
    }

    componentWillReceiveProps(nextProps: Props) {
        if(this.props.selectedAircraftID !== nextProps.selectedAircraftID) {
            this.state.selectedAircraftID = nextProps.selectedAircraftID;
        }
    }

    handleDropdownChange = (event: any, index: number, value: string) => {
        this.setState({selectedAircraftID: value});
        this.props.onSelectedAircraftChange(value);
    }


    render() {

        const width = window.screen.width;
        // const height = window.screen.height;
        const aircraftCommsContainer = {
            // backgroundColor: colors.orange700,
            position: 'absolute',
            bottom: 0,
            left: 50 + '%',
            // height: 64,
            zIndex: 9999,
            width: 50 + "%",
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            marginLeft: -width*0.25
        };
        const buttonStyle = { margin: 5 };

        let vehicleIDs: JSX.Element[] = [];
        for( let key in this.props.connectedVehicles ){
            // let vehicle = this.props.connectedVehicles[key];

            vehicleIDs.push(
                <MenuItem key={key} value={key} primaryText={key} label={key} />
            );
        }

        return(
            <div>
                {Object.keys(this.props.connectedVehicles).length >= 0 &&
                    <div style={aircraftCommsContainer}>
                            {Object.keys(this.props.connectedVehicles).length > 1 &&
                                <MuiThemeProvider muiTheme={lightMuiTheme}>
                                    <DropDownMenu style={{marginRight: 10, width: 150, backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedAircraftID} onChange={this.handleDropdownChange}>
                                        <MenuItem value={"0"} primaryText={"All vehicles"} label={"All vehicles"} />
                                        {vehicleIDs}
                                    </DropDownMenu>
                                </MuiThemeProvider>
                            }

                            {this.props.connectedVehicles[this.props.selectedAircraftID] &&
                                <MuiThemeProvider muiTheme={lightMuiTheme}>
                                    {this.props.connectedVehicles[this.props.selectedAircraftID].isArmed ?
                                        <RaisedButton icon={<i className="material-icons">clear</i>} style={buttonStyle} label="Disarm" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_ARM", JSON.stringify({arm: false}))}/>
                                        :
                                        <RaisedButton icon={<i className="material-icons">check</i>} style={buttonStyle} label="Arm" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_ARM", JSON.stringify({arm: true}))}/>
                                    }
                                </MuiThemeProvider>
                            }

                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">flight_takeoff</i>} style={buttonStyle} label="Takeoff" onClick={this.props.handleTakeoff}/>
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">pause</i>} style={buttonStyle} label="Loiter" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "LOITER")}/>
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">get_app</i>} style={buttonStyle} label="Land" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "LAND")}/>
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">home</i>} style={buttonStyle} label="Home" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "RTL")}/>
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">send</i>} style={buttonStyle} label="Start Mission" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "AUTO")}/>
                            </MuiThemeProvider>
                    </div>
                }
            </div>
        )

    }
}
