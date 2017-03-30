import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Drawer from 'material-ui/Drawer';
import FlatButton from 'material-ui/FlatButton';
import MenuItem from 'material-ui/MenuItem';
import FontIcon from 'material-ui/FontIcon';
import Divider from 'material-ui/Divider';


type Props = {
    openDrawer: boolean,
    onToggleDrawer: (open: boolean) => void,
    onDrawerAction: (action: string) => void,
    showMessagesMenu: boolean
}

type State = {
}

export class AppDrawer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }


    render() {

        const drawerItemLabelStyle = { fontSize: 18, marginLeft: 20 };

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Drawer
                containerStyle={{position: "absolute", top: 64}}
                docked={true}
                open={this.props.openDrawer}
                onRequestChange={(open: boolean) => this.props.onToggleDrawer(open)}
                >

                    <div style={{ textAlign: "center" }}>
                        <img className="image" src={"images/HeronLogo_Small.png"} />
                    </div>

                    <Divider />

                    <MenuItem onTouchTap={() => this.props.onDrawerAction("TestButton")}>
                        <FlatButton
                        labelStyle={drawerItemLabelStyle}
                        label="TESTING"
                        icon={<FontIcon className="material-icons">gesture</FontIcon>}
                        />
                    </MenuItem>

                    {/*
                    <MenuItem onTouchTap={() => this.props.onDrawerAction("Logging")}>
                        <FlatButton
                        labelStyle={drawerItemLabelStyle}
                        label="Logging"
                        icon={<FontIcon className="material-icons">event_note</FontIcon>}
                        />
                    </MenuItem>
                    */}

                    <Divider />

                    <MenuItem onTouchTap={() => this.props.onDrawerAction("Settings")}>
                        <FlatButton
                        labelStyle={drawerItemLabelStyle}
                        label="Settings"
                        icon={<FontIcon className="material-icons">settings</FontIcon>}
                        />
                    </MenuItem>
                </Drawer>
            </MuiThemeProvider>
        )

    }
}
