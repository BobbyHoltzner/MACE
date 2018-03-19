import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import TextField from 'material-ui/TextField';
import { Vehicle } from '../../../util/Vehicle/Vehicle';
import { Grid, Col } from 'react-bootstrap';
import SelectField from 'material-ui/SelectField';
import MenuItem from 'material-ui/MenuItem';

import * as colors from 'material-ui/styles/colors';
import * as L from 'leaflet';
// import { styles } from "./styles";


type Props = {
    vehicles: {[id: string]: Vehicle},
    selectedVehicleID: string,
    open: boolean,
    handleClose: () => void,
    handleSave: (vehicleID: string, vehicleHome: PositionType) => void,
    contextAnchor: L.LeafletMouseEvent,
    useContext: boolean,
    allowVehicleSelect: boolean,
    onSelectedAircraftChange: (id: string) => void,
    showNotification: (title: string, message: string, level: string, position: string, label: string) => void
}

type State = {
    selectedVehicleID?: string,
    homeLat?: number,
    homeLon?: number,
    homeAlt?: number
}

export class VehicleHomeDialog extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        let homeLat = this.props.vehicles[this.props.selectedVehicleID] ? this.props.vehicles[this.props.selectedVehicleID].homePosition.latLon.lat : 0;
        let homeLon = this.props.vehicles[this.props.selectedVehicleID] ? this.props.vehicles[this.props.selectedVehicleID].homePosition.latLon.lng : 0;
        if(this.props.useContext) {
            homeLat = this.props.contextAnchor.latlng.lat;
            homeLon = this.props.contextAnchor.latlng.lng;
        }

        this.state = {
            selectedVehicleID: this.props.selectedVehicleID,
            homeLat: homeLat,
            homeLon: homeLon,
            homeAlt: 0
        }
    }

    componentWillReceiveProps(nextProps: Props) {
        if(nextProps.selectedVehicleID !== this.props.selectedVehicleID) {
            this.setState({selectedVehicleID: nextProps.selectedVehicleID});
        }

        if(nextProps.useContext) {
            this.setState({
                homeLat: this.props.contextAnchor.latlng.lat,
                homeLon: this.props.contextAnchor.latlng.lng
            })
        }

    }

    handleTextChange = (event: any) => {
        this.setState({[event.target.id]: event.target.value});
    }

    handleSave = () => {
        console.log("Selected vehicle ID: " + this.state.selectedVehicleID);
        let vehicleHome: PositionType = {
            lat: this.state.homeLat,
            lng: this.state.homeLon,
            alt: this.state.homeAlt
        }
        if(this.state.selectedVehicleID !== '0') {
            this.props.handleSave(this.state.selectedVehicleID, vehicleHome);
            this.props.handleClose();
        }
        else {
            let title = 'Set Home';
            let level = 'info';
            this.props.showNotification(title, 'Select a vehicle to set home coordinates for.', level, 'tc', 'Got it');
        }
    }

    handleDropdownChange = (event: any, index: number, value: string) => {
        this.setState({selectedVehicleID: value});
        this.props.onSelectedAircraftChange(value);
    }

    render() {
        const actions = [
            <FlatButton
                label="Cancel"
                onTouchTap={this.props.handleClose}
            />,
            <FlatButton
                label="Save"
                labelStyle={{color: colors.orange700}}
                onTouchTap={this.handleSave}
            />,
        ];

        let vehicleIDs: JSX.Element[] = [];
        for( let key in this.props.vehicles ){
            // let vehicle = this.props.connectedVehicles[key];
            vehicleIDs.push(
                <MenuItem key={key} value={key} primaryText={key} label={key} />
            );
        }

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog titleStyle={{backgroundColor: colors.orange700, color: colors.white}} title="Set vehicle home position" actions={actions} modal={false} open={this.props.open} onRequestClose={this.props.handleClose} contentStyle={{width: '20%'}}>
                    <Grid fluid>
                        <Col xs={12} md={12}>
                                <MuiThemeProvider muiTheme={lightMuiTheme}>
                                    <SelectField floatingLabelText="Select a vehicle" style={{marginRight: 10, width: '100%', backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedVehicleID} onChange={this.handleDropdownChange}>
                                        <MenuItem value={"0"} primaryText={"Select a vehicle..."} label={"Select a vehicle..."} />
                                        {vehicleIDs}
                                    </SelectField>
                                </MuiThemeProvider>
                        </Col>
                        <Col xs={12} md={12}>
                            <TextField
                                id={"homeLat"}
                                floatingLabelText="Latitude (decimal)"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                onChange={this.handleTextChange}
                                type={"number"}
                                value={this.state.homeLat}
                            />
                        </Col>
                        <Col xs={12} md={12}>
                            <TextField
                                id={"homeLon"}
                                floatingLabelText="Longitude (decimal)"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                onChange={this.handleTextChange}
                                type={"number"}
                                value={this.state.homeLon}
                            />
                        </Col>
                        {/* NOT SETTING ALTITUDE AT THIS TIME--DEFAULTED TO 0
                        <Col xs={12}>
                            <TextField
                                floatingLabelText="Altitude"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                type={"number"}
                                value={this.state.homeAlt}
                            />
                        </Col>
                        */}
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        )

    }
}
