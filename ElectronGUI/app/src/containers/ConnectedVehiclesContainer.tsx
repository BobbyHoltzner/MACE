import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import FlatButton from 'material-ui/FlatButton'

import { VehicleHUD } from '../components/VehicleHUD';
import { Vehicle } from '../Vehicle';


type Props = {
    connectedVehicles: {[id: string]: Vehicle},
    onAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void,
    handleOpenVehicleEdit: (vehicleID: string) => void
    selectedVehicleID: string
}

type State = {
}

export class ConnectedVehiclesContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
        }
    }

    handleAircraftCommand = (vehicleID: string, tcpCommand: string, vehicleCommand: string) => {
        console.log("Command: " + vehicleCommand + " for vehicleID: " + vehicleID);
        this.props.onAircraftCommand(vehicleID, tcpCommand, vehicleCommand);
    }

    render() {

        const height = window.screen.height;
        const connectedVehiclesContainer = { position: 'absolute', height: height, right: 0, zIndex: 999, width: 20 + "%", backgroundColor: 'rgba(255,255,255,1)', display: 'flex', alignItems: 'center', flexDirection: 'column', maxHeight: height, overflowY: "scroll" };
        // const connectedVehiclesContainer = { position: 'absolute', height: height, right: 0, zIndex: 999, width: 20 + "%", backgroundColor: 'rgba(153,153,153,.2)', display: 'flex', alignItems: 'center', flexDirection: 'column', maxHeight: height, overflowY: "scroll" };
        // const openButtonContainer = { position: 'absolute', top: 15, right: 15, zIndex: 999, backgroundColor: "rgba(255,255,255,1)" };

        let vehicleHUDs: JSX.Element[] = [];
        for( let key in this.props.connectedVehicles ){
            let vehicle = this.props.connectedVehicles[key];

            vehicleHUDs.push(
                <VehicleHUD key={key}
                    vehicleID={key}
                    aircraft={vehicle}
                    handleAircraftCommand={this.handleAircraftCommand}
                    handleOpenVehicleEdit={this.props.handleOpenVehicleEdit}
                />
            );
        }

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div>
                    {Object.keys(this.props.connectedVehicles).length > 0 ?
                        <div style={connectedVehiclesContainer}>
                            <div>
                                <FlatButton
                                    label="Sync"
                                    labelPosition="after"
                                    onClick={() => this.handleAircraftCommand("0", "GET_CONNECTED_VEHICLES", "")}
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
