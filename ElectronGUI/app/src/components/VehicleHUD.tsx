import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import {Card, CardActions, CardHeader, CardText} from 'material-ui/Card';
import FlatButton from 'material-ui/FlatButton';
import { Vehicle } from '../Vehicle';
import { backgroundColors, textSeverityToColor } from '../util/Colors';


type Props = {
    vehicleID: string,
    aircraft: Vehicle,
    handleAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void
    handleChangeSelectedVehicle: (vehicleID: string) => void
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
        const hudAvatar = this.props.aircraft.vehicleType === "Copter" ? "images/drone-icon.png" : "images/fixed-wing.png";

        const hudTitle = <div className="row">
                            <div className="col-xs-6">
                                <div className="col-xs-12">
                                    <span>ID: {this.props.vehicleID}</span>
                                </div>
                                <div className="col-xs-12">
                                    <span style={{fontSize: 14+'px', color: 'rgba(0, 0, 0, 0.541176)'}}>{this.props.aircraft.vehicleMode}</span>
                                </div>
                            </div>
                            <div className="col-xs-6" style={{paddingLeft: 0, paddingRight: 0}}>
                                <div className="col-xs-12" style={{paddingLeft: 0, paddingRight: 0}}>
                                    <div className="col-xs-6" style={{display: 'flex', justifyContent: 'flex-end', alignItems: 'center', paddingLeft: 0, paddingRight: 0}}>
                                        <i className="material-icons">gps_fixed</i>
                                    </div>
                                    <div className="col-xs-6" style={{display: 'flex', justifyContent: 'center', alignItems: 'center', paddingLeft: 0, paddingRight: 0, paddingTop: 4+'px'}}>
                                        <span style={{fontSize: 12}}>{this.props.aircraft.gps.gpsFix}</span>
                                    </div>
                                </div>
                                <div className="col-xs-12" style={{paddingLeft: 0, paddingRight: 0}}>
                                    <div className="col-xs-6" style={{display: 'flex', justifyContent: 'flex-end', alignItems: 'center', paddingLeft: 0, paddingRight: 0}}>
                                        <i className="material-icons">satellite</i>
                                    </div>
                                    <div className="col-xs-6" style={{display: 'flex', justifyContent: 'center', alignItems: 'center', paddingLeft: 0, paddingRight: 0, paddingTop: 4+'px'}}>
                                        <span style={{fontSize: 12}}>{this.props.aircraft.gps.visibleSats}</span>
                                    </div>
                                </div>
                            </div>
                         </div>

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card expanded={true} onExpandChange={() => this.props.handleChangeSelectedVehicle(this.props.vehicleID)} style={hudStyle}>
                    <CardHeader
                        titleStyle={{fontSize: 24}}
                        title={hudTitle}
                        avatar={hudAvatar}
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
                                    {"Airspeed: " + this.props.aircraft.airspeed.toFixed(2)}
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

                        {this.props.aircraft.messages.length > 0 &&
                            <div className="col-xs-12">
                                <br></br>
                                <div style={{width: '100%', height: '9px', borderBottom: '1px solid #8c8b8b', textAlign: 'center'}}>
                                    <span style={{backgroundColor: '#fff', padding: '0 10px'}}>
                                        Messages
                                    </span>
                                </div>
                                <br></br>
                                <div className="box">
                                    <CardText style={{color: textSeverityToColor(this.props.aircraft.messages[0].severity), fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                        {'[' + this.props.aircraft.messages[0].severity + '] : ' + this.props.aircraft.messages[0].text}
                                    </CardText>
                                </div>
                                <br></br>
                                <div style={{width: '100%', borderBottom: '1px solid #8c8b8b'}}></div>
                            </div>
                        }

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
