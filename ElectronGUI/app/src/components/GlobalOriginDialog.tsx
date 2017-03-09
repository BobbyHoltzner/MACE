import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import Dialog from 'material-ui/Dialog';
import FlatButton from 'material-ui/FlatButton';
import TextField from 'material-ui/TextField';
import { Grid, Col } from 'react-bootstrap';

import * as colors from 'material-ui/styles/colors';


type Props = {
    open: boolean,
    handleClose: () => void,
    onGlobalHomeCommand: (vehicleID: string, tcpCommand: string, vehicleCommand: string) => void,
    globalOrigin: PositionType,
    handleSave: (vehicleHome: PositionType) => void,
    contextAnchor: L.LeafletMouseEvent,
    useContext: boolean
}

type State = {
    globalLat?: number,
    globalLon?: number,
    globalAlt?: number
}

export class GlobalOriginDialog extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);

        this.state = {
            globalLat: this.props.useContext ? this.props.contextAnchor.latlng.lat : 0,
            globalLon: this.props.useContext ? this.props.contextAnchor.latlng.lng : 0,
            globalAlt: 0
        }
    }

    componentWillReceiveProps(nextProps: Props) {
        if(nextProps.useContext) {
            this.setState({
                globalLat: this.props.contextAnchor.latlng.lat,
                globalLon: this.props.contextAnchor.latlng.lng
            })
        }

    }

    handleTextChange = (event: any) => {
        this.setState({[event.target.id]: event.target.value});
    }

    handleSave = () => {
        let globalHome: PositionType = {
            lat: this.state.globalLat,
            lon: this.state.globalLon,
            alt: this.state.globalAlt
        }
        this.props.handleSave(globalHome);
        this.props.handleClose();
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

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Dialog titleStyle={{backgroundColor: colors.orange700, color: colors.white}} title="Set global home position" actions={actions} modal={false} open={this.props.open} onRequestClose={this.props.handleClose}>
                    <Grid fluid>
                        <Col xs={12} md={6}>
                            <TextField
                                id={"globalLat"}
                                floatingLabelText="Latitude (decimal)"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                onChange={this.handleTextChange}
                                value={this.state.globalLat}
                            />
                        </Col>
                        <Col xs={12} md={6}>
                            <TextField
                                id={"globalLon"}
                                floatingLabelText="Longitude (decimal)"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                onChange={this.handleTextChange}
                                value={this.state.globalLon}
                            />
                        </Col>
                        {/* NOT SETTING ALTITUDE AT THIS TIME--DEFAULTED TO 0
                        <Col xs={12}>
                            <TextField
                                floatingLabelText="Altitude"
                                floatingLabelFocusStyle={{color: colors.orange700}}
                                underlineFocusStyle={{borderColor: colors.orange700}}
                                value={this.state.globalAlt}
                            />
                        </Col>
                        */}
                    </Grid>
                </Dialog>
            </MuiThemeProvider>
        )

    }
}