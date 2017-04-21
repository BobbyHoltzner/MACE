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
    openSettingsSubmenu?: boolean
}

export class AppDrawer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            openSettingsSubmenu: false
        }
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

                    <MenuItem onTouchTap={() => this.props.onDrawerAction("TestButton1")}>
                        <FlatButton
                            labelStyle={drawerItemLabelStyle}
                            label="TESTING 1"
                            icon={<FontIcon className="material-icons">gesture</FontIcon>}
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={'rgba(0,0,0, 0.0)'}
                        />
                    </MenuItem>

                    <MenuItem onTouchTap={() => this.props.onDrawerAction("TestButton2")}>
                        <FlatButton
                            labelStyle={drawerItemLabelStyle}
                            label="TESTING 2"
                            icon={<FontIcon className="material-icons">gesture</FontIcon>}
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={'rgba(0,0,0, 0.0)'}
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

                    <MenuItem onTouchTap={() => this.setState({openSettingsSubmenu: !this.state.openSettingsSubmenu})}>
                        <FlatButton
                            labelStyle={drawerItemLabelStyle}
                            label="Settings"
                            labelPosition="before"
                            icon={this.state.openSettingsSubmenu ? <FontIcon className="material-icons">expand_less</FontIcon> : <FontIcon className="material-icons">expand_more</FontIcon>}
                            disableTouchRipple={false}
                            disableFocusRipple={false}
                            hoverColor={'rgba(0,0,0, 0.0)'}
                        />
                        {this.state.openSettingsSubmenu &&
                            <div>
                                <MenuItem onTouchTap={() => this.props.onDrawerAction("Messages")}>
                                    <FlatButton
                                        labelStyle={drawerItemLabelStyle}
                                        label="Messages"
                                        icon={<FontIcon className="material-icons">event_note</FontIcon>}
                                        disableTouchRipple={false}
                                        disableFocusRipple={false}
                                        hoverColor={'rgba(0,0,0, 0.0)'}
                                    />
                                </MenuItem>
                                <MenuItem onTouchTap={() => this.props.onDrawerAction("Takeoff")}>
                                    <FlatButton
                                        labelStyle={drawerItemLabelStyle}
                                        label="Takeoff"
                                        icon={<FontIcon className="material-icons">flight_takeoff</FontIcon>}
                                        disableTouchRipple={false}
                                        disableFocusRipple={false}
                                        hoverColor={'rgba(0,0,0, 0.0)'}
                                    />
                                </MenuItem>
                            </div>
                        }
                    </MenuItem>
                </Drawer>
            </MuiThemeProvider>
        )

    }
}
