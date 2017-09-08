import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import Checkbox from 'material-ui/Checkbox';
import { Grid, Col } from 'react-bootstrap';
import * as deepcopy from 'deepcopy';
import TextField from 'material-ui/TextField';
import { Colors } from '../util/Colors';

import * as colors from 'material-ui/styles/colors';


type Props = {
    open: boolean,
    handleClose: () => void,
    handleSave: (sliderSettings: EnvironmentSettingsType) => void
    environmentSettings: EnvironmentSettingsType
}

type State = {
    environmentSettings?: EnvironmentSettingsType,
    minSliderVal?: number,
    maxSliderVal?: number,
    showBoundingBox?: boolean,
    defaultGridSpacing?: number
}

export class EnvironmentSettings extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            environmentSettings: deepcopy(this.props.environmentSettings),
            minSliderVal: this.props.environmentSettings.minSliderVal,
            maxSliderVal: this.props.environmentSettings.maxSliderVal,
            showBoundingBox: this.props.environmentSettings.showBoundingBox,
            defaultGridSpacing: this.props.environmentSettings.gridSpacing
        }
    }

    handleSave = () => {
        let settings: EnvironmentSettingsType = {
            minSliderVal: this.state.minSliderVal,
            maxSliderVal: this.state.maxSliderVal,
            showBoundingBox: this.state.showBoundingBox,
            gridSpacing: this.state.defaultGridSpacing
        }
        this.setState({environmentSettings: settings});
        this.props.handleSave(settings);
        this.props.handleClose();
    }

    handleCancel = () => {
        let tmpPreferences = deepcopy(this.props.environmentSettings);
        this.setState({
            environmentSettings: tmpPreferences,
            minSliderVal: tmpPreferences.minSliderVal,
            maxSliderVal: tmpPreferences.maxSliderVal,
            showBoundingBox: tmpPreferences.showBoundingBox,
            defaultGridSpacing: tmpPreferences.gridSpacing
        });
        this.props.handleClose();
    }

    handleTextChange = (event: any) => {
        this.setState({[event.target.id]: parseInt(event.target.value)});
    }

    handleCheck = (checked: boolean, preference: string) => {
        let settings: any = deepcopy(this.state.environmentSettings);
        settings[preference] = checked;
        this.setState({environmentSettings: settings});
    }

    render() {

        const checkboxStyle = {paddingTop: 24};
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
                <Dialog titleStyle={{backgroundColor: colors.orange700, color: colors.white}} title="Environment Drawing Settings" actions={actions} modal={false} open={this.props.open} onRequestClose={this.handleCancel}>
                    <Grid style={{marginTop: 35}} fluid>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"minSliderVal"}
                                    floatingLabelText="Min Grid Spacing"
                                    floatingLabelFocusStyle={{color: colors.orange700}}
                                    underlineFocusStyle={{borderColor: colors.orange700}}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.minSliderVal.toString()}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"maxSliderVal"}
                                    floatingLabelText="Max grid spacing"
                                    floatingLabelFocusStyle={{color: colors.orange700}}
                                    underlineFocusStyle={{borderColor: colors.orange700}}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.maxSliderVal.toString()}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"defaultGridSpacing"}
                                    floatingLabelText="Default grid spacing"
                                    floatingLabelFocusStyle={{color: colors.orange700}}
                                    underlineFocusStyle={{borderColor: colors.orange700}}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.defaultGridSpacing.toString()}
                                />
                            </Col>
                        </Col>
                        <Col xs={12} md={6}>
                            <Col xs={12} md={12}>
                                <Checkbox
                                    label="Show bounding box"
                                    labelStyle={{color: Colors.Primary, fontWeight: 'normal'}}
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "showBoundingBox")}
                                    checked={this.state.environmentSettings.showBoundingBox}
                                />
                            </Col>
                        </Col>
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        )

    }
}
