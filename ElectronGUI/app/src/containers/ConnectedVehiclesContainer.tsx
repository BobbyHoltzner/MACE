import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import FlatButton from 'material-ui/FlatButton'

import { VehicleHUD, VehicleMapType } from '../components/VehicleHUD';


type Props = {
    connectedVehicles: VehicleMapType
}

type State = {
    selectedVehicle?: string,
    showConnectedVehicles?: boolean
}

export class ConnectedVehiclesContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            selectedVehicle: "0",
            showConnectedVehicles: true
        }
    }


    toggleContainerCollapse = () => {
        this.setState({showConnectedVehicles: !this.state.showConnectedVehicles});
    }
    handleAircraftCommand = (vehicleID: string, command: string) => {
        console.log("Command: " + command + " for vehicleID: " + vehicleID);
    }

    render() {
        
        const height = window.screen.height;
        const connectedVehiclesContainer = { position: 'absolute', height: height, right: 0, zIndex: 999, width: 20 + "%", backgroundColor: 'rgba(255,255,255,1)', display: 'flex', alignItems: 'center', flexDirection: 'column', maxHeight: height, overflowY: "scroll" };   
        const openButtonContainer = { position: 'absolute', top: 15, right: 15, zIndex: 999, backgroundColor: "rgba(255,255,255,1)" };

        let vehicleHUDs: JSX.Element[] = [];
        for( let key in this.props.connectedVehicles ){
            let vehicle = this.props.connectedVehicles[key];

            vehicleHUDs.push(
                <VehicleHUD key={key}
                    vehicleID={key}
                    aircraft={vehicle}
                    isSelected={this.state.selectedVehicle === key ? true : false}
                    handleAircraftCommand={this.handleAircraftCommand}
                />
            );
        }

        return(               
            <MuiThemeProvider muiTheme={lightMuiTheme}>   
                <div>                  
                    {Object.keys(this.props.connectedVehicles).length > 0 &&
                        <div style={openButtonContainer}>
                            <FlatButton
                                label="Show"
                                onClick={this.toggleContainerCollapse}
                                icon={<i className="material-icons">keyboard_arrow_left</i>}
                            />
                        </div>
                    }                                            
                    {(this.state.showConnectedVehicles && Object.keys(this.props.connectedVehicles).length > 0) ? 
                        <div style={connectedVehiclesContainer}>
                            <div>
                                <FlatButton
                                    label="Hide"
                                    labelPosition="after"
                                    onClick={this.toggleContainerCollapse}
                                    icon={<i className="material-icons">keyboard_arrow_right</i>}
                                />
                            </div>

                            {vehicleHUDs}
                                                        
                        </div>
                     : null
                    }

                </div>
            </MuiThemeProvider>            
        )

    }
}
