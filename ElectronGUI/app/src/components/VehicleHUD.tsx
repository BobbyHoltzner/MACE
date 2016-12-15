import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import {Card, CardActions, CardHeader, CardText} from 'material-ui/Card';
import FlatButton from 'material-ui/FlatButton';

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
    handleAircraftCommand: (vehicleID: number, command: string) => void
}

type State = {
}

export class VehicleHUD extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }

    handleLoiter = () => {
        this.props.handleAircraftCommand(this.props.aircraft.id, "Loiter");
    }
    handleRTL = () => {
        this.props.handleAircraftCommand(this.props.aircraft.id, "RTL");
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
                        {"Lat: " + this.props.aircraft.position.lat} 
                    </CardText>
                    <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                        {"Lon: " + this.props.aircraft.position.lon} 
                    </CardText>
                    <CardText style={{fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                        {"Alt: " + this.props.aircraft.position.alt} 
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
