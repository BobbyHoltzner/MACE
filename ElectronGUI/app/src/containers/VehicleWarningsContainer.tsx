import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';

export type VehicleWarning = {
    vehicleID: number,
    warning: string
}

type Props = {
    vehicleWarnings: VehicleWarning[]
}

type State = {
}

export class VehicleWarningsContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }


    render() {

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <div></div>
            </MuiThemeProvider>
        )

    }
}
