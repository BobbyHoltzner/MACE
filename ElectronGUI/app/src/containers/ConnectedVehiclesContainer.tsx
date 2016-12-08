import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';

import { VehicleHUD, VehicleType } from '../components/VehicleHUD';


type Props = {
    connectedVehicles: VehicleType[]
}

type State = {
    selectedVehicle?: VehicleType
}

export class ConnectedVehiclesContainer extends React.Component<Props, State> {

    constructor() {
        super();

        this.state = {
            selectedVehicle: null
        }
    }


    render() {

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <VehicleHUD
                    aircraft={this.state.selectedVehicle} 
                />
            </MuiThemeProvider>
        )

    }
}
