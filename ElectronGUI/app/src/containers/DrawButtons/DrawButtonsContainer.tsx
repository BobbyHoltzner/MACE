import * as materialColors from 'material-ui/styles/colors';
import { Colors } from '../../util/misc/Colors';
import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme({
  slider: {
    trackSize: 5,
    handleSize: 20,
    trackColor: materialColors.white,
    trackColorSelected: materialColors.white,
    selectionColor: Colors.Primary,
    rippleColor: Colors.Primary
  }
});
import * as React from 'react';
import RaisedButton from 'material-ui/RaisedButton';
import FontIcon from 'material-ui/FontIcon';
import Slider from 'material-ui/Slider';
import IconButton from 'material-ui/IconButton';
import { styles } from "./styles";


type Props = {
    onDeleteLastPolygonPt: () => void,
    onDisableDraw: () => void,
    onSubmitBoundary: () => void,
    onClearAllPts: () => void,
    handleChangeGridSpacing: (val: number) => void,
    openEnvironmentSettings: () => void,
    environmentSettings: EnvironmentSettingsType
}

type State = {
    sliderVal?: number,
    minSliderVal?: number,
    maxSliderVal?: number
}

export class DrawButtonsContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
        this.state = {
            sliderVal: this.props.environmentSettings.gridSpacing,
            minSliderVal: this.props.environmentSettings.minSliderVal,
            maxSliderVal: this.props.environmentSettings.maxSliderVal
        }
    }

    componentWillReceiveProps(nextProps: Props) {
        this.setState({sliderVal: nextProps.environmentSettings.gridSpacing});
    }

    handleChange = (event: any, newValue: number) => {
        this.setState({sliderVal: newValue});
    }

    onDragStop = (event: any) => {
        this.props.handleChangeGridSpacing(this.state.sliderVal);
    }


    // onMinValChange = (e: any) => {
    //     console.log(e.target.value);
    //     this.setState({minSliderVal: e.target.value});
    // }

    // onMaxValChange = (e: any) => {
    //     console.log("MAX Value: " + e);
    // }

    openEnvSettings = () => {
        console.log("Open settings...");
        this.props.openEnvironmentSettings();
    }

    render() {

        // <i className="material-icons">battery_charging_full</i>

        return(
            <div style={styles.parentContainer}>
                <div style={styles.buttonsContainer}>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton
                            onClick={this.props.onSubmitBoundary}
                            icon={<FontIcon className="material-icons" color={materialColors.black}>check</FontIcon>}
                            label={"Submit"}
                            style={{marginTop: 5, width: 120}}
                        />
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton
                            onClick={this.props.onDeleteLastPolygonPt}
                            icon={<FontIcon className="material-icons" color={materialColors.black}>undo</FontIcon>}
                            label={"Undo"}
                            style={{marginTop: 5, width: 120}}
                        />
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton
                            onClick={this.props.onClearAllPts}
                            icon={<FontIcon className="material-icons" color={materialColors.black}>delete</FontIcon>}
                            label={"Clear"}
                            style={{marginTop: 5, width: 120}}
                        />
                    </MuiThemeProvider>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton
                            onClick={this.props.onDisableDraw}
                            icon={<FontIcon className="material-icons" color={materialColors.black}>clear</FontIcon>}
                            label={"Exit"}
                            style={{marginTop: 20, width: 120}}
                        />
                    </MuiThemeProvider>
                </div>
                <div style={styles.sliderContainer}>
                    <div style={styles.inlineContainer}>
                        <p style={{color: materialColors.white, textAlign: 'center', fontSize: 18}}>Grid spacing: {this.state.sliderVal} meters</p>
                        <IconButton onClick={this.openEnvSettings} style={{paddingBottom: 10, padding: 0, color: materialColors.white}}><i className="material-icons">settings</i></IconButton>
                    </div>
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <Slider
                            value={(this.state.sliderVal >= this.props.environmentSettings.minSliderVal && this.state.sliderVal <= this.props.environmentSettings.maxSliderVal) ? this.state.sliderVal : this.state.minSliderVal}
                            defaultValue={(this.props.environmentSettings.gridSpacing >= this.props.environmentSettings.minSliderVal && this.props.environmentSettings.gridSpacing <= this.props.environmentSettings.maxSliderVal) ? this.props.environmentSettings.gridSpacing : this.props.environmentSettings.minSliderVal}
                            onChange={this.handleChange}
                            onDragStop={this.onDragStop}
                            min={this.props.environmentSettings.minSliderVal}
                            max={this.props.environmentSettings.maxSliderVal}
                            step={1}
                            sliderStyle={{padding: 0, marginBottom: 12, marginTop: 12}}
                        />
                    </MuiThemeProvider>
                    <div style={styles.inlineContainer}>
                        <p style={{color: materialColors.white, textAlign: 'center', fontSize: 18}}>Min: {this.props.environmentSettings.minSliderVal} m</p>
                        <p style={{color: materialColors.white, textAlign: 'center', fontSize: 18}}>Max: {this.props.environmentSettings.maxSliderVal} m</p>
                    </div>
                    {/* <Checkbox
                                    label="Emergency"
                                    labelStyle={{color: textSeverityToColor('EMERGENCY')}}
                                    style={checkboxStyle}
                                    onCheck={(event: any, checked: boolean) => this.handleCheck(checked, "emergency")}
                                    checked={this.state.environmentSettings.emergency}
                                    /> */}
                </div>
            </div>
        )

    }
}
