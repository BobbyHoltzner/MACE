import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import TextField from 'material-ui/TextField';
import SelectField from 'material-ui/SelectField';
import MenuItem from 'material-ui/MenuItem';
import { Vehicle } from '../../../util/Vehicle/Vehicle';
import { Grid, Col } from 'react-bootstrap';

import * as colors from 'material-ui/styles/colors';
import * as L from 'leaflet';


type Props = {
    vehicles: {[id: string]: Vehicle},
    selectedVehicleID: string,
    open: boolean,
    handleClose: () => void,
    handleTakeoff: (vehicleID: string, takeoffAlt: string, takeoffLat?: string, takeoffLon?: string) => void,
    takeoffAlt: string,
    onSelectedAircraftChange: (id: string) => void,
    showSaveTakeoff: boolean,
    handleSaveTakeoff: (alt: string) => void,
    contextAnchor: L.LeafletMouseEvent,
    useContext: boolean,
    showNotification: (title: string, message: string, level: string, position: string, label: string) => void
}

type State = {
    selectedVehicleID?: string,
    takeoffAlt?: string,
    takeoffLat?: string,
    takeoffLon?: string
}

export class TakeoffDialog extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            selectedVehicleID: this.props.selectedVehicleID,
            takeoffAlt: this.props.takeoffAlt,
            takeoffLat: this.props.useContext ? this.props.contextAnchor.latlng.lat.toString() : "0",
            takeoffLon: this.props.useContext ? this.props.contextAnchor.latlng.lng.toString() : "0"
        }
    }

    componentWillReceiveProps(nextProps: Props) {
        if(nextProps.selectedVehicleID !== this.props.selectedVehicleID) {
            this.setState({selectedVehicleID: nextProps.selectedVehicleID});
        }
    }

    handleTextChange = (event: any) => {
        this.setState({[event.target.id]: event.target.value});
    }

    handleTakeoff = () => {
        if(this.props.useContext) {
            if(this.state.selectedVehicleID !== "0") {
                this.props.handleTakeoff(this.state.selectedVehicleID, this.state.takeoffAlt, this.state.takeoffLat, this.state.takeoffLon);
            }
            else {
                let title = 'Takeoff';
                let level = 'info';
                this.props.showNotification(title, 'Select a vehicle to send takeoff coordinates to.', level, 'tc', 'Got it');
                return;
            }
        }
        else {
            this.props.handleTakeoff(this.state.selectedVehicleID, this.state.takeoffAlt);
        }
        this.props.handleClose();
    }

    handleSaveTakeoff = () => {
        this.props.handleSaveTakeoff(this.state.takeoffAlt);
        this.props.handleClose();
    }

    handleDropdownChange = (event: any, index: number, value: string) => {
        this.setState({selectedVehicleID: value});
        this.props.onSelectedAircraftChange(value);
    }

    render() {

        let actions: JSX.Element[] = [];
        {this.props.showSaveTakeoff ?
            actions = [
                <FlatButton
                    label="Cancel"
                    onTouchTap={this.props.handleClose}
                />,
                <FlatButton
                    label="Save takeoff"
                    labelStyle={{color: colors.orange700}}
                    onTouchTap={this.handleSaveTakeoff}
                />,
            ]
            :
            actions = [
                <FlatButton
                    label="Cancel"
                    onTouchTap={this.props.handleClose}
                />,
                <FlatButton
                    label="Takeoff"
                    labelStyle={{color: colors.orange700}}
                    onTouchTap={this.handleTakeoff}
                />,
            ]
        }


        let vehicleIDs: JSX.Element[] = [];
        for( let key in this.props.vehicles ){
            // let vehicle = this.props.connectedVehicles[key];
            vehicleIDs.push(
                <MenuItem key={key} value={key} primaryText={key} label={key} />
            );
        }

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog titleStyle={{backgroundColor: colors.orange700, color: colors.white}} title="Set vehicle takeoff altitude" actions={actions} modal={false} open={this.props.open} onRequestClose={this.props.handleClose} contentStyle={{width: '20%'}}>
                    {this.props.showSaveTakeoff ?
                        <Grid fluid>
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"takeoffAlt"}
                                    floatingLabelText="Takeoff altitude"
                                    floatingLabelFocusStyle={{color: colors.orange700}}
                                    underlineFocusStyle={{borderColor: colors.orange700}}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.takeoffAlt}
                                />
                            </Col>
                        </Grid>
                        :
                        <Grid fluid>
                            <Col xs={12} md={12}>
                                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                                        <SelectField floatingLabelText="Select a vehicle" style={{marginRight: 10, width: '100%', backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedVehicleID} onChange={this.handleDropdownChange}>
                                            <MenuItem value={"0"} primaryText={this.props.useContext ? "Select a vehicle..." : "All vehicles"} label={this.props.useContext ? "Select a vehicle..." : "All vehicles"} />
                                            {vehicleIDs}
                                        </SelectField>
                                    </MuiThemeProvider>
                            </Col>
                            {this.props.useContext &&
                                <Col xs={12} md={12}>
                                    <Col xs={12} md={12}>
                                        <TextField
                                            id={"takeoffLat"}
                                            floatingLabelText="Takeoff latitude"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.takeoffLat}
                                        />
                                    </Col>
                                    <Col xs={12} md={12}>
                                        <TextField
                                            id={"takeoffLon"}
                                            floatingLabelText="Takeoff longitude"
                                            floatingLabelFocusStyle={{color: colors.orange700}}
                                            underlineFocusStyle={{borderColor: colors.orange700}}
                                            onChange={this.handleTextChange}
                                            type={"number"}
                                            value={this.state.takeoffLon}
                                        />
                                    </Col>
                                </Col>
                            }
                            <Col xs={12} md={12}>
                                <TextField
                                    id={"takeoffAlt"}
                                    floatingLabelText="Takeoff altitude"
                                    floatingLabelFocusStyle={{color: colors.orange700}}
                                    underlineFocusStyle={{borderColor: colors.orange700}}
                                    onChange={this.handleTextChange}
                                    type={"number"}
                                    value={this.state.takeoffAlt}
                                />
                            </Col>
                        </Grid>
                    }
                </Dialog>
            </MuiThemeProvider>
        )

    }
}
