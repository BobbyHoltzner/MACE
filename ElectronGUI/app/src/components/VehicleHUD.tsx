import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';

export type VehicleType = {
    id: number,
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

type Props = {
    aircraft: VehicleType
}

type State = {
}

export class VehicleHUD extends React.Component<Props, State> {

    constructor() {
        super();
    }


    render() {

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div></div>
            </MuiThemeProvider>
        )

    }
}
