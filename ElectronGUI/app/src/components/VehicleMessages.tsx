import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as React from 'react';
import {Card, CardActions, CardHeader, CardText} from 'material-ui/Card';
import FlatButton from 'material-ui/FlatButton';
import { Vehicle } from '../Vehicle';
import { backgroundColors, textSeverityToColor } from '../util/Colors';


type Props = {
    vehicleID: string,
    aircraft: Vehicle
}

type State = {
}

export class VehicleMessages extends React.Component<Props, State> {

    constructor(props: Props) {
        super(props);
    }

    handleClear = () => {
        this.props.aircraft.messages = [];
    }

    render() {
        const boxShadow = this.props.aircraft.isSelected ? backgroundColors[parseInt(this.props.vehicleID)] + " 0px 1px 20px, rgba(0, 0, 0, .5) 0px 1px 4px" : "rgba(0, 0, 0, 0.117647) 0px 1px 6px, rgba(0, 0, 0, 0.117647) 0px 1px 4px"
        const hudStyle = {position: "relative", width: 90 + "%", marginBottom: 15, boxShadow: boxShadow};
        const hudAvatar = this.props.aircraft.vehicleType === "Copter" ? "images/drone-icon.png" : "images/fixed-wing.png";

        let messages: JSX.Element[] = [];
        for( let key in this.props.aircraft.messages ){
            let textColor = textSeverityToColor(this.props.aircraft.messages[key].severity);
            messages.push(
                <CardText key={key} style={{color: textColor, fontSize: 18, paddingTop: 0, paddingBottom: 0}}>
                    {'[' + this.props.aircraft.messages[key].severity + '] : ' + this.props.aircraft.messages[key].text}
                </CardText>
            );
        }

        return(
            <MuiThemeProvider muiTheme={lightMuiTheme}>
                <Card expanded={true} style={hudStyle}>
                    <CardHeader
                        titleStyle={{fontSize: 24}}
                        title={"ID: " + this.props.vehicleID}
                        subtitle={this.props.aircraft.vehicleMode}
                        avatar={hudAvatar}
                        actAsExpander={true}
                        showExpandableButton={false}
                    />

                    <div className="row">
                        <div className="col-xs-12">
                            <div className="box" style={{overflowY: 'scroll', maxHeight: '150px'}}>
                                {messages}
                            </div>
                        </div>
                    </div>

                    <CardActions style={{textAlign: "center"}}>
                        <FlatButton
                            label="Clear"
                            onClick={this.handleClear}
                            icon={<i className="material-icons">delete_sweep</i>} />
                    </CardActions>

                </Card>
            </MuiThemeProvider>
        )

    }
}
