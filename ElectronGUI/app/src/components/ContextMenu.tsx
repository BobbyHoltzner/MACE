import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Popover from 'material-ui/Popover';
import Menu from 'material-ui/Menu';
import MenuItem from 'material-ui/MenuItem';

// import * as colors from 'material-ui/styles/colors';


type Props = {
    menuAnchor: L.LeafletMouseEvent,
    handleClose: () => void,
    handleSetHome: () => void,
    handleSetGlobal: () => void
}

type State = {
    xPos?: number,
    yPos?: number
}

export class ContextMenu extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            xPos: this.props.menuAnchor ? this.props.menuAnchor.containerPoint.x : 0,
            yPos: this.props.menuAnchor ? this.props.menuAnchor.containerPoint.y : 0
        }
    }

    handleSetHome = () => {
        this.props.handleSetHome();
        this.props.handleClose();
    }

    handleSetGlobal = () => {
        this.props.handleSetGlobal();
        this.props.handleClose();
    }

    render() {

        const menuStyle = {
            position: "absolute",
            left: this.state.xPos,
            top: this.state.yPos+65,
            width: 200,
            backgroundColor: "#ffffff"
        }

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Menu style={menuStyle}>
                    <MenuItem primaryText="Set home location" onClick={this.handleSetHome} />
                    <MenuItem primaryText="Set global origin" onClick={this.handleSetGlobal} />
                </Menu>
            </MuiThemeProvider>
        )

    }
}
