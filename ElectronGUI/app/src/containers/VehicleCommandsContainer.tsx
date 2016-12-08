import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';


type Props = {
}

type State = {
}

export class VehicleCommandsContainer extends React.Component<Props, State> {

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
