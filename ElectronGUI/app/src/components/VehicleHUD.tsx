import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import {Card, CardActions, CardHeader, CardText} from 'material-ui/Card';
import FlatButton from 'material-ui/FlatButton';
// import deepEqual from '../helpers/DeepEqual';

// export function generateNewVehicle(): VehicleStateType {
//     let vehicleState = {
//         position: {
//             lat: 0,
//             lon: 0,
//             alt: 0,
//             numSats: 0,
//             positionFix: 0
//         },
//         attitude: {
//             roll: 0,
//             pitch: 0,
//             yaw: 0
//         }
//     };
//     return vehicleState;
// }


type Props = {
    vehicleID: string,
    aircraft: VehicleStateType,
    isSelected: boolean,
    handleAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void
}

type State = {
}

export class VehicleHUD extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }

    // shouldComponentUpdate(nextProps: Props, nextState: State){
    //     console.log("This props: " + this.props.aircraft.attitude);
    //     console.log("Next props: " + nextProps.aircraft.attitude);
    //     if(!deepEqual(this.props.aircraft.attitude, nextProps.aircraft.attitude)) {
    //         console.log('In deep equal');
    //         return true;
    //     }

    //     if(this.props.aircraft.attitude.yaw !== nextProps.aircraft.attitude.yaw){
    //         console.log("In deep equal 2");
    //         return true;
    //     }

    //     return false;
    // }

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

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card expanded={true} onExpandChange={() => console.log("TEST CLICK")} style={{position: "relative", width: 90 + "%", marginBottom: 15}}>
                    <CardHeader
                        titleStyle={{fontSize: 24}}
                        title={"ID: " + this.props.vehicleID}
                        subtitle={"Vehicle type??"}
                        avatar={"images/drone-icon.png"}
                        actAsExpander={true}
                        showExpandableButton={false}
                    />

                    <div className="row">
                        <div className="col-xs-6">
                            <div className="box">
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Lat: " + this.props.aircraft.position.lat.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Lon: " + this.props.aircraft.position.lon.toFixed(2)}
                                </CardText>
                                <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                                    {"Alt: " + this.props.aircraft.position.alt.toFixed(2)}
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
