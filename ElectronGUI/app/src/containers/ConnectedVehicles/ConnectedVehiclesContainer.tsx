import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import FlatButton from 'material-ui/FlatButton'

import { VehicleHUD } from '../../components/VehicleHUD/VehicleHUD';
import { VehicleMessages } from '../../components/VehicleMessages/VehicleMessages';
import { Vehicle } from '../../util/Vehicle/Vehicle';
import { styles } from "./styles";


type Props = {
    connectedVehicles: {[id: string]: Vehicle},
    onAircraftCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void,
    handleChangeSelectedVehicle: (vehicleID: string) => void
    selectedVehicleID: string
}

type State = {
    showHUDs?: boolean
}

export class ConnectedVehiclesContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            showHUDs: true
        }
    }

    handleAircraftCommand = (vehicleID: string, tcpCommand: string, vehicleCommand: string) => {
        console.log("Command: " + vehicleCommand + " for vehicleID: " + vehicleID);
        this.props.onAircraftCommand(vehicleID, tcpCommand, vehicleCommand);
    }

    render() {

        let vehicleHUDs: JSX.Element[] = [];
        let vehicleMessages: JSX.Element[] = [];
        for( let key in this.props.connectedVehicles ){
            let vehicle = this.props.connectedVehicles[key];
            vehicleHUDs.push(
                <VehicleHUD key={key}
                    vehicleID={key}
                    aircraft={vehicle}
                    handleAircraftCommand={this.handleAircraftCommand}
                    handleChangeSelectedVehicle={this.props.handleChangeSelectedVehicle}
                    highlightColor={this.props.connectedVehicles[key].highlightColor}
                />
            );

            vehicleMessages.push(
                <VehicleMessages key={key}
                    vehicleID={key}
                    aircraft={vehicle}
                />
            )
        }


        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div>
                    {Object.keys(this.props.connectedVehicles).length > 0 ?
                        <div style={styles.connectedVehiclesContainer}>
                            <div>
                                <FlatButton
                                    label={this.state.showHUDs ? "Vehicle Messages" : "Vehicle HUDs"}
                                    labelPosition="after"
                                    onClick={() => this.setState({showHUDs: !this.state.showHUDs})}
                                    icon={this.state.showHUDs ? <i className="material-icons">keyboard_arrow_right</i> : <i className="material-icons">keyboard_arrow_left</i>}
                                />
                            </div>
                            {this.state.showHUDs ? vehicleHUDs : vehicleMessages}
                        </div>
                     : null
                    }

                </div>
            </MuiThemeProvider>
        )

    }
}
