import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import {Card, CardActions, CardHeader, CardText} from 'material-ui/Card';
import FlatButton from 'material-ui/FlatButton';
import { Vehicle } from '../Vehicle';
import { backgroundColors } from '../util/Colors';


type Props = {
    vehicleID: string,
    aircraft: Vehicle,
    handleAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void
    handleOpenVehicleEdit: (vehicleID: string) => void
}

type State = {
}

export class VehicleHUD extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }

    handleLoiter = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "SET_VEHICLE_MODE", "LOITER");
    }
    handleRTL = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "SET_VEHICLE_MODE", "RTL");
    }
    syncVehicle = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "GET_VEHICLE_HOME", "");
        this.props.handleAircraftCommand(this.props.vehicleID, "GET_VEHICLE_MISSION", "");
    }

    render() {
        const boxShadow = this.props.aircraft.isSelected ? backgroundColors[parseInt(this.props.vehicleID)] + " 0px 1px 20px, rgba(0, 0, 0, .5) 0px 1px 4px" : "rgba(0, 0, 0, 0.117647) 0px 1px 6px, rgba(0, 0, 0, 0.117647) 0px 1px 4px"
        const hudStyle = {position: "relative", width: 90 + "%", marginBottom: 15, boxShadow: boxShadow};

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card expanded={true} onExpandChange={() => this.props.handleOpenVehicleEdit(this.props.vehicleID)} style={hudStyle}>
                    <CardHeader
                        titleStyle={{fontSize: 24}}
                        title={"ID: " + this.props.vehicleID}
                        subtitle={this.props.aircraft.vehicleType}
                        avatar={"images/drone-icon.png"}
                        actAsExpander={true}
                        showExpandableButton={false}
                    />

                    <div className="row">
                        <div className="col-xs-6">
                            <div className="box">
                                {/*
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Lat: " + this.props.aircraft.position.lat.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Lon: " + this.props.aircraft.position.lon.toFixed(2)}
                                </CardText>
                                */}
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Alt: " + this.props.aircraft.position.alt.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Batt: " + this.props.aircraft.fuel.batteryRemaining.toFixed(0) + "%"}
                                </CardText>
                            </div>
                        </div>
                        <div className="col-xs-6">
                            <div className="box">
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Roll: " + this.props.aircraft.attitude.roll.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Pitch: " + this.props.aircraft.attitude.pitch.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Yaw: " + this.props.aircraft.attitude.yaw.toFixed(2)}
                                </CardText>
                            </div>
                        </div>
                    </div>


                    <CardActions style={{textAlign: "center"}}>
                        <FlatButton
                            label="Loiter"
                            onClick={this.handleLoiter}
                            icon={<i className="material-icons">pause</i>} />
                        <FlatButton
                            label="RTL"
                            onClick={this.handleRTL}
                            icon={<i className="material-icons">home</i>} />
                        <FlatButton
                            label="Sync"
                            onClick={this.syncVehicle}
                            icon={<i className="material-icons">swap_calls</i>} />
                    </CardActions>

                </Card>
            </MuiThemeProvider>
        )

    }
}
