import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import TextField from 'material-ui/TextField';
import { Grid, Col } from 'react-bootstrap';
import {Tabs, Tab} from 'material-ui/Tabs';
import * as deepcopy from 'deepcopy';

import * as colors from 'material-ui/styles/colors';
// import { styles } from "./styles";


type Props = {
    open: boolean,
    handleClose: () => void,
    handleSave: (configSettings: ConfigSettingsType, reload: boolean) => void,
    configSettings: ConfigSettingsType,
    handleParseJSON: (filename: string, restartServer: boolean) => void
}

type State = {
    configSettings?: ConfigSettingsType,
    activeTab?: string
}

export class ConfigDialog extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            configSettings: deepcopy(this.props.configSettings),
            activeTab: "browse"
        }
    }

    shouldComponentUpdate(nextProps: Props, state: State) {
        this.setState({configSettings: deepcopy(nextProps.configSettings)});
        return true;
        // TODO-PAT: Not sure I like having to do this...
    }


   compareObjects = (obj1: any, obj2: any) => {
        //Loop through properties in object 1
        for (var p in obj1) {
            //Check property exists on both objects
            if (obj1.hasOwnProperty(p) !== obj2.hasOwnProperty(p)) {
                console.log("**COMPARE - NO PROPERTY: " + p);
                return false;
            }
            switch (typeof (obj1[p])) {
                //Deep compare objects
                case 'object':
                    if (!this.compareObjects(obj1[p], obj2[p])){
                        console.log("**COMPARE - DEEP COMPARE FALSE: " + p);
                        return false;
                    }
                    break;
                //Compare function code
                case 'function':
                    if (typeof (obj2[p]) == 'undefined' || (p != 'compare' && obj1[p].toString() != obj2[p].toString())) {
                        console.log("**COMPARE - function: " + p);
                        return false;
                    }
                    break;
                //Compare values
                default:
                    if (obj1[p] != obj2[p]) {
                        console.log("**COMPARE - value: " + p);
                        return false;
                    }
            }
        }

        //Check object 2 for any extra properties
        for (var p in obj2) {
            if (typeof (obj1[p]) == 'undefined') {
                console.log("**COMPARE - extra props: " + p);
                return false;
            }
        }

        console.log("**COMPARE - EQUAL");
        return true;
    };

    handleSave = (reload: boolean = false) => {
        this.props.handleSave(this.state.configSettings, reload);
        this.props.handleClose();
    }

    handleCancel = () => {
        let tmpConfigSettings = deepcopy(this.props.configSettings);
        this.setState({configSettings: tmpConfigSettings});
        this.props.handleClose();
    }

    handleTextChange = (event: any) => {
        let config: ConfigSettingsType = deepcopy(this.state.configSettings);
        // if(event.target.id === 'filename') {
        //     config.filename = event.target.value;
        // }
        if(event.target.id === 'ipAddress') {
            config.config.MACEComms.ipAddress = event.target.value;
        }
        else if(event.target.id === 'listenPortNumber') {
            config.config.MACEComms.listenPortNumber = event.target.value;
        }
        else if(event.target.id === 'sendPortNumber') {
            config.config.MACEComms.sendPortNumber = event.target.value;
        }
        else if(event.target.id === 'mapCenterLat') {
            config.config.GUIInit.mapCenter.lat = event.target.value;
        }
        else if(event.target.id === 'mapCenterLng') {
            config.config.GUIInit.mapCenter.lng = event.target.value;
        }
        else if(event.target.id === 'mapZoom') {
            config.config.GUIInit.mapZoom = event.target.value;
        }
        else if(event.target.id === 'maxZoom') {
            config.config.GUIInit.maxZoom = event.target.value;
        }
        else if(event.target.id === 'defaultTakeoffAlt') {
            config.config.VehicleSettings.defaultTakeoffAlt = event.target.value;
        }

        this.setState({configSettings: config});
    }

    handleChangeTab = (tab: any) => {
        // console.log(tab);
        // console.log(`A tab with this route property ${tab.props['label']} was activated.`);
        this.setState({activeTab: tab.props.value});
    }

    chooseFileChange = (e: any) => {
        let config: ConfigSettingsType = deepcopy(this.state.configSettings);
        config.filename = e.target.files[0].path;
        this.setState({configSettings: config});
        this.props.handleParseJSON(config.filename, false);
    }

    render() {

        let actions = [];
        if(this.state.activeTab !== "vehicle") {
            actions = [
                <FlatButton
                    label="Cancel"
                    onTouchTap={this.handleCancel}
                />,
                <FlatButton
                    label="Save and reload"
                    labelStyle={{color: colors.orange700}}
                    onTouchTap={() => this.handleSave(true)}
                />,
            ];
        }
        else {
            actions = [
                <FlatButton
                    label="Cancel"
                    onTouchTap={this.handleCancel}
                />,
                <FlatButton
                    label="Save"
                    labelStyle={{color: colors.orange700}}
                    onTouchTap={() => this.handleSave()}
                />,
            ];
        }

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog titleStyle={{backgroundColor: colors.orange700, color: colors.white}} title="MACE Config" actions={actions} modal={false} open={this.props.open} onRequestClose={this.handleCancel} contentStyle={{width: '30%'}}>
                    <Grid style={{marginTop: 35}} fluid>
                        <Col xs={12} md={12}>
                            <Tabs value={this.state.activeTab}>
                                <Tab value="browse" onActive={this.handleChangeTab} label="Browse" style={{backgroundColor: colors.white, color: colors.orange700}}>
                                    <Col xs={12} md={12} style={{marginTop: 20, display: 'flex', justifyContent: 'center'}}>
                                        {/* <TextField
                                            id={"filename"}
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"file"}
                                            value={this.state.configSettings.filename}
                                            style={{marginTop: 20}}
                                            underlineShow={false}
                                            inputStyle={{fontSize: 20}}
                                        /> */}

                                        <i className="material-icons" style={{color: "black"}}>file_upload</i>
                                        <input type="file" name="file" id="file" className="inputfile" onChange={this.chooseFileChange} />
                                        <label htmlFor="file">{this.state.configSettings.filename == '' ? "Choose a file..." : this.state.configSettings.filename}</label>
                                    </Col>
                                </Tab>
                                <Tab value="comms" onActive={this.handleChangeTab} label="Comms" style={{backgroundColor: colors.white, color: colors.orange700}}>
                                    <Col xs={12} md={12} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"ipAddress"}
                                            floatingLabelText="IP address"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            value={this.state.configSettings.config.MACEComms.ipAddress}
                                        />
                                    </Col>
                                    <Col xs={12} md={12} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"listenPortNumber"}
                                            floatingLabelText="Listen port"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.MACEComms.listenPortNumber}
                                        />
                                    </Col>
                                    <Col xs={12} md={12} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"sendPortNumber"}
                                            floatingLabelText="Send port"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.MACEComms.sendPortNumber}
                                        />
                                    </Col>
                                </Tab>
                                <Tab value="map" onActive={this.handleChangeTab} label="Map" style={{backgroundColor: colors.white, color: colors.orange700}}>
                                    <Col xs={6} md={6} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"mapCenterLat"}
                                            floatingLabelText="Map center latitude"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.GUIInit.mapCenter.lat}
                                            style={{width: "90%"}}
                                        />
                                    </Col>
                                    <Col xs={6} md={6} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"mapCenterLng"}
                                            floatingLabelText="Map center longitude"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.GUIInit.mapCenter.lng}
                                            style={{width: "90%"}}
                                        />
                                    </Col>

                                    <Col xs={12} md={12} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"mapZoom"}
                                            floatingLabelText="Default map zoom"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.GUIInit.mapZoom}
                                        />
                                    </Col>

                                    <Col xs={12} md={12} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"maxZoom"}
                                            floatingLabelText="Max zoom level"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.GUIInit.maxZoom}
                                        />
                                    </Col>
                                </Tab>
                                <Tab value="vehicle" onActive={this.handleChangeTab} label="Vehicle" style={{backgroundColor: colors.white, color: colors.orange700}}>
                                    <Col xs={12} md={12} style={{display: 'flex', justifyContent: 'center'}}>
                                        <TextField
                                            id={"defaultTakeoffAlt"}
                                            floatingLabelText="Default Takeoff altitude"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.configSettings.config.VehicleSettings.defaultTakeoffAlt}
                                        />
                                    </Col>
                                </Tab>
                            </Tabs>

                        </Col>
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        )
    }
}
