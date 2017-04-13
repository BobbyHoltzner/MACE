import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import Checkbox from 'material-ui/Checkbox';
import { Grid, Col } from 'react-bootstrap';
import * as deepcopy from 'deepcopy';

import * as colors from 'material-ui/styles/colors';


type Props = {
    open: boolean,
    handleClose: () => void,
    handleSave: (preferences: MessagePreferencesType) => void
    preferences: MessagePreferencesType
}

type State = {
    preferences?: MessagePreferencesType
}

export class MessagesDialog extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            preferences: deepcopy(this.props.preferences)
        }
    }

    handleSave = () => {
        this.props.handleSave(this.state.preferences);
        this.props.handleClose();
    }

    handleCheck = (checked: boolean, preference: string) => {
        let stateVar: any = deepcopy(this.state.preferences);
        stateVar[preference] = checked;
        this.setState({preferences: stateVar});
    }

    handleCancel = () => {
        this.state.preferences = deepcopy(this.props.preferences);
        this.props.handleClose();
    }

    render() {

        const checkboxStyle = {marginBottom: 16};
        const actions = [
            <FlatButton
                label="Cancel"
                onTouchTap={this.handleCancel}
            />,
            <FlatButton
                label="Save"
                labelStyle={{color: colors.orange700}}
                onTouchTap={this.handleSave}
            />,
        ];

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog titleStyle={{backgroundColor: colors.orange700, color: colors.white}} title="Message Display Preferences" actions={actions} modal={false} open={this.props.open} onRequestClose={this.handleCancel}>
                    <Grid style={{marginTop: 35}} fluid>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Emergency"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "emergency")}
                                    checked={this.state.preferences.emergency}
                                    />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Critical"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "critical")}
                                    checked={this.state.preferences.critical}
                                    />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Alert"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "alert")}
                                    checked={this.state.preferences.alert}
                                    />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Error"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "error")}
                                    checked={this.state.preferences.error}
                                    />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Warning"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "warning")}
                                    checked={this.state.preferences.warning}
                                    />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Notice"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "notice")}
                                    checked={this.state.preferences.notice}
                                    />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Info"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "info")}
                                    checked={this.state.preferences.info}
                                    />
                            </Col>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Debug"
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "debug")}
                                    checked={this.state.preferences.debug}
                                    />
                            </Col>
                        </Col>
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        )

    }
}
