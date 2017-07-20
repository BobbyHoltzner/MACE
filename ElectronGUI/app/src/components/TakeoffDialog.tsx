import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import TextField from 'material-ui/TextField';
import DropDownMenu from 'material-ui/DropDownMenu';
import MenuItem from 'material-ui/MenuItem';
import { Vehicle } from '../Vehicle';
import { Grid, Col } from 'react-bootstrap';

import * as colors from 'material-ui/styles/colors';


type Props = {
    vehicles: {[id: string]: Vehicle},
    selectedVehicleID: string,
    open: boolean,
    handleClose: () => void,
    handleTakeoff: (vehicleID: string, takeoffAlt: number) => void,
    takeoffAlt: number,
    onSelectedAircraftChange: (id: string) => void,
    showSaveTakeoff: boolean,
    handleSaveTakeoff: (alt: number) => void
}

type State = {
    selectedVehicleID?: string,
    takeoffAlt?: number
}

export class TakeoffDialog extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            selectedVehicleID: this.props.selectedVehicleID,
            takeoffAlt: this.props.takeoffAlt
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
        this.props.handleTakeoff(this.state.selectedVehicleID, this.state.takeoffAlt);
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
                            {this.props.selectedVehicleID === "0" &&
                                <Col xs={12} md={12}>
                                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                                            <DropDownMenu style={{marginRight: 10, width: '100%', backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedVehicleID} onChange={this.handleDropdownChange}>
                                                <MenuItem value={"0"} primaryText={"All vehicles"} label={"All vehicles"} />
                                                {vehicleIDs}
                                            </DropDownMenu>
                                        </MuiThemeProvider>
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
