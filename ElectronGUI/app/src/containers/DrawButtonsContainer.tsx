import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import RaisedButton from 'material-ui/RaisedButton';
import FontIcon from 'material-ui/FontIcon';
import * as colors from 'material-ui/styles/colors';


type Props = {
    onDeleteLastPolygonPt: () => void,
    onDisableDraw: () => void,
    onSubmitBoundary: () => void,
    onClearAllPts: () => void
}

type State = {
}

export class DrawButtonsContainer extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }

    render() {

        const drawButtonsContainer = {
            position: 'absolute' as 'absolute',
            top: 80,
            left: 26,
            display: "flex",
            justifyContent: "space-between" as "space-between",
            alignItems: "center" as "center",
            flexDirection: "column" as "column",
        };

        // <i className="material-icons">battery_charging_full</i>

        return(
            <div style={drawButtonsContainer}>
                <MuiThemeProvider muiTheme={lightMuiTheme}>
                    <RaisedButton
                        onClick={this.props.onSubmitBoundary}
                        icon={<FontIcon className="material-icons" color={colors.black}>check</FontIcon>}
                        label={"Submit"}
                        style={{marginTop: 5, width: 120}}
                    />
                </MuiThemeProvider>
                <MuiThemeProvider muiTheme={lightMuiTheme}>
                    <RaisedButton
                        onClick={this.props.onDeleteLastPolygonPt}
                        icon={<FontIcon className="material-icons" color={colors.black}>undo</FontIcon>}
                        label={"Undo"}
                        style={{marginTop: 5, width: 120}}
                    />
                </MuiThemeProvider>
                <MuiThemeProvider muiTheme={lightMuiTheme}>
                    <RaisedButton
                        onClick={this.props.onClearAllPts}
                        icon={<FontIcon className="material-icons" color={colors.black}>delete</FontIcon>}
                        label={"Clear"}
                        style={{marginTop: 5, width: 120}}
                    />
                </MuiThemeProvider>
                <MuiThemeProvider muiTheme={lightMuiTheme}>
                    <RaisedButton
                        onClick={this.props.onDisableDraw}
                        icon={<FontIcon className="material-icons" color={colors.black}>clear</FontIcon>}
                        label={"Exit"}
                        style={{marginTop: 20, width: 120}}
                    />
                </MuiThemeProvider>
            </div>
        )

    }
}
