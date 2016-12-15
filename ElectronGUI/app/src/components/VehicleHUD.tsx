import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import {Card, CardActions, CardHeader, CardText} from 'material-ui/Card';
import FlatButton from 'material-ui/FlatButton';
// import deepEqual from '../helpers/DeepEqual';

export type VehicleStateType = {
    position: {
        lat: number,
        lon: number,
        alt: number
    },
    attitude: {
        roll: number,
        pitch: number,
        yaw: number
    }
}

export function generateNewVehicle(): VehicleStateType {
    let vehicleState = {
        position: {
            lat: 0,
            lon: 0,
            alt: 0
        },
        attitude: {
            roll: 0,
            pitch: 0,
            yaw: 0
        }
    };
    return vehicleState;
}

export type VehicleMapType = {[id: string]: VehicleStateType};


type Props = {
    vehicleID: string,
    aircraft: VehicleStateType,
    isSelected: boolean,
    handleAircraftCommand: (vehicleID: string, command: string) => void
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
        this.props.handleAircraftCommand(this.props.vehicleID, "Loiter");
    }
    handleRTL = () => {
        this.props.handleAircraftCommand(this.props.vehicleID, "RTL");
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
                    <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                        {"Roll: " + this.props.aircraft.attitude.roll.toFixed(2)} 
                    </CardText>
                    <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                        {"Pitch: " + this.props.aircraft.attitude.pitch.toFixed(2)} 
                    </CardText>
                    <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                        {"Yaw: " + this.props.aircraft.attitude.yaw.toFixed(2)} 
                    </CardText>
                    <CardActions style={{textAlign: "center"}}>
                        <FlatButton 
                            label="Loiter" 
                            onClick={this.handleLoiter}
                            icon={<i className="material-icons">pause</i>} />
                        <FlatButton 
                            label="RTL" 
                            onClick={this.handleRTL}
                            icon={<i className="material-icons">home</i>} />
                    </CardActions>
                    
                </Card>
            </MuiThemeProvider>
        )

    }
}
