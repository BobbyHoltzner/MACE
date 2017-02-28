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
    onSelectedAircraftChange: (id: string) => void,
    onAircraftCommand: (id: string, tcpCommand: string, vehicleCommand: string) => void
}

type State = {
    selectedAircraftID?: string
}

export class VehicleCommandsContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            selectedAircraftID: "1"
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
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <DropDownMenu style={{marginRight: 10, width: 150, backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedAircraftID} onChange={this.handleDropdownChange}>
                                    <MenuItem value={"0"} primaryText={"1"} label={"id 1"} />
                                    <MenuItem value={"1"} primaryText={"2"} label={"id 2"} />
                                    {vehicleIDs}
                                </DropDownMenu>
                            </MuiThemeProvider>

                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">flight_takeoff</i>} style={buttonStyle} label="Launch" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "TAKEOFF")}/>

                                {/*
                                <RaisedButton style={buttonStyle} onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "GUIDED")} >
                                    <Grid>
                                        <Row>
                                            <Col xs={12}>
                                                <FontIcon className="material-icons">home</FontIcon>
                                            </Col>
                                            <Col xs={12}>
                                                <div>Launch</div>
                                            </Col>
                                        </Row>
                                    </Grid>
                                </RaisedButton>
                                */}
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">get_app</i>} style={buttonStyle} label="Land" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "LAND")}/>
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">home</i>} style={buttonStyle} label="Home" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "SET_VEHICLE_MODE", "RTL")}/>
                            </MuiThemeProvider>
                            <MuiThemeProvider muiTheme={lightMuiTheme}>
                                <RaisedButton icon={<i className="material-icons">send</i>} style={buttonStyle} label="Send WPs" onClick={() => this.props.onAircraftCommand(this.state.selectedAircraftID.toString(), "GET_CONNECTED_VEHICLES", "")}/>
                            </MuiThemeProvider>
                    </div>
                }
            </div>
        )

    }
}
