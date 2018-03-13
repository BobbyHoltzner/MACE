import * as React from 'react';

import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();

var NotificationSystem = require('react-notification-system');
import { ConnectedVehiclesContainer } from './ConnectedVehiclesContainer';
import { VehicleCommandsContainer } from './VehicleCommandsContainer';
import { DrawButtonsContainer } from './DrawButtonsContainer';
import { EnvironmentSettings } from '../components/EnvironmentSettings';
import { AppDrawer } from './AppDrawer';
import AppBar from 'material-ui/AppBar';
import * as colors from 'material-ui/styles/colors';
import { Vehicle } from '../Vehicle';
import { AppHelper } from '../util/AppHelper';
import { VehicleHomeDialog } from '../components/VehicleHomeDialog';
import { GlobalOriginDialog } from '../components/GlobalOriginDialog';
import { MessagesDialog } from '../components/MessagesDialog';
import { ConfigDialog } from '../components/ConfigDialog';
import { TakeoffDialog } from '../components/TakeoffDialog';
import MACEMap from '../components/MACEMap';
import FlatButton from 'material-ui/FlatButton';
import * as deepcopy from 'deepcopy';

var injectTapEventPlugin = require("react-tap-event-plugin");
injectTapEventPlugin();

import * as L from 'leaflet';

// // Performance testing:
// var Perf = require('react-addons-perf');
// // End performance testing

type Props = {
}

type State = {
  selectedVehicleID?: string,
  openDrawer?: boolean,
  showEditVehicleHomeDialog?: boolean,
  showEditGlobalHomeDialog?: boolean,
  showMessagesMenu?: boolean,
  showConfigDialog?: boolean,
  showTakeoffDialog?: boolean,
  showSaveTakeoff?: boolean,
  showEnvironmentSettings?: boolean,
  getConnectedVehiclesTimeout?: number,
  useContext?: boolean,
  contextAnchor?: L.LeafletMouseEvent,
  connectedVehicles?: {[id: string]: Vehicle}
}

export default class AppContainer extends React.Component<Props, State> {
  notificationSystem: any; // TODO: Figure out why I cant make this a NotificationSystem type...
  getVehiclesInterval: any;
  logger: any;
  appHelper: AppHelper;

  constructor(props: Props) {
    super(props);

    this.getVehiclesInterval = null;

    this.state = {
      selectedVehicleID: "0",
      openDrawer: false,
      showEditVehicleHomeDialog: false,
      showEditGlobalHomeDialog: false,
      showMessagesMenu: false,
      showConfigDialog: false,
      showTakeoffDialog: false,
      showSaveTakeoff: false,
      showEnvironmentSettings: false,
      getConnectedVehiclesTimeout: 3000,
      useContext: false,
      connectedVehicles: {}
    }


    this.appHelper = new AppHelper(this.state);
  }

  componentDidMount(){
    // // Performance testing:
    // setTimeout(() => {
    //   console.log("Start performance testing...");
    //   Perf.start();
    //   setTimeout(() => {
    //     Perf.stop();
    //     const measurements = Perf.getLastMeasurements();
    //     Perf.printWasted(measurements);
    //   }, 30000);
    // }, 5000);
    // // End performance testing

    this.notificationSystem = this.refs.notificationSystem;

    this.appHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");

    this.getVehiclesInterval = setInterval(() => {
      this.appHelper.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
    }, this.appHelper.getConnectedVehiclesTimeout);


    // Set interval to set state to DB:
    setInterval(() => {
      if(!this.appHelper.pauseMACEComms) {
          this.setState({connectedVehicles: this.appHelper.vehicleDB});
      }
    }, 1500);
  }

  showNotification = (title: string, message: string, level: string, position: string, label: string) => {
    let notification = {
      title: title,
      message: message,
      level: level,
      position: position,
      action: {
        label: label
      }
    }

    this.notificationSystem.addNotification(notification);
  }

  handleAircraftCommand = (id: string, tcpCommand: string, vehicleCommand: string) => {
    console.log(tcpCommand);
    this.appHelper.makeTCPRequest(parseInt(id), tcpCommand, vehicleCommand);
  }

  handleDrawerAction = (action: string) => {
    if(action === "MACEConfig"){
      this.setState({showMessagesMenu: false, showConfigDialog: true, showTakeoffDialog: false, showSaveTakeoff: false, openDrawer: false});
      this.appHelper.pauseMACEComms = true;
    }
    else if(action === "Messages"){
      this.setState({showMessagesMenu: true, showConfigDialog: false,  showTakeoffDialog: false, showSaveTakeoff: false, openDrawer: false});
      this.appHelper.pauseMACEComms = false;
    }
    else if(action === "Takeoff"){
      this.setState({showMessagesMenu: false, showConfigDialog: false,  showTakeoffDialog: true, showSaveTakeoff: true, openDrawer: false});
      this.appHelper.pauseMACEComms = false;
    }
    else if(action === "TestButton1") {
      this.appHelper.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION1", "");
    }
    else if(action === "TestButton2") {
      this.appHelper.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION2", "");
    }
    else if(action === "EditEnvironment") {
      // Ask for global origin:
      this.appHelper.makeTCPRequest(0, "GET_GLOBAL_ORIGIN", "");

      this.setState({openDrawer: false});
      this.appHelper.showDraw = true;
    }
  }

  contextSetHome = () => {
    this.setState({
      showEditVehicleHomeDialog: true,
      showEditGlobalHomeDialog: false,
      showTakeoffDialog: false,
      useContext: true
    });
    this.appHelper.allowVehicleSelect = true;
    this.appHelper.pauseMACEComms = true;
  }

  contextSetGlobal = () => {
    this.setState({
      showEditGlobalHomeDialog: true,
      showEditVehicleHomeDialog: false,
      useContext: true
    });
    this.appHelper.allowVehicleSelect = false;
    this.appHelper.pauseMACEComms = true;
  }

  contextSetTakeoff = () => {
    this.setState({
      showEditVehicleHomeDialog: false,
      showEditGlobalHomeDialog: false,
      showTakeoffDialog: true,
      useContext: true,
    });
    this.appHelper.allowVehicleSelect = false;
    this.appHelper.pauseMACEComms = true;
  }

  contextGoHere = () => {
    this.setState({
      showEditGlobalHomeDialog: false,
      showEditVehicleHomeDialog: false,
      useContext: true
    });
    this.appHelper.allowVehicleSelect = false;
    let goHere = {
      lat: this.state.contextAnchor.latlng.lat,
      lon: this.state.contextAnchor.latlng.lng
    };
    this.handleAircraftCommand(this.state.selectedVehicleID, "SET_GO_HERE", JSON.stringify(goHere));
  }

  handleSelectedAircraftUpdate = (id: string) => {
    let stateCopy = deepcopy(this.state.connectedVehicles);
    let selectedID = "0";
    Object.keys(this.state.connectedVehicles).map((key: string) => {
      if(key === id){
        stateCopy[id].isSelected = !stateCopy[id].isSelected;
        selectedID = stateCopy[id].isSelected ? id : "0";

        if(stateCopy[id].isSelected === true && (stateCopy[id].position.lat !== 0 && stateCopy[id].position.lat !== 0)){
          // this.setState({mapCenter: [stateCopy[id].position.lat, stateCopy[id].position.lon]});
        }
      }
      else {
        stateCopy[key].isSelected = false;
      }
      stateCopy[key].updateHomePosition();
      stateCopy[key].updateVehicleMarkerPosition();
    });

    // this.vehicleDB = stateCopy;
    // this.setState({connectedVehicles: stateCopy, selectedVehicleID: selectedID});
    this.appHelper.vehicleDB = stateCopy;
    this.setState({selectedVehicleID: selectedID});
  }

  updateMapCenter = (e: L.DragEndEvent) => {
    // let MACEconfig = this.state.MACEconfig;
    // MACEconfig.config.GUIInit.mapCenter = {lat: e.target.getCenter().lat, lng: e.target.getCenter().lng, alt: 0};
    // MACEconfig.config.GUIInit.mapZoom = e.target.getZoom();
    this.appHelper.updateMapCenter(e);
    // this.setState({MACEconfig: this.appHelper.MACEconfig});
  }

  handleSaveTakeoff = (takeoffAlt: string) => {
    // let MACEconfig = this.state.MACEconfig;
    // MACEconfig.config.VehicleSettings.defaultTakeoffAlt = parseFloat(takeoffAlt);
    this.appHelper.handleSaveTakeoff(takeoffAlt);
    // this.setState({MACEconfig: this.appHelper.MACEconfig});
  }

  handleTakeoff = () => {
    this.setState({
      showTakeoffDialog: true,
      showSaveTakeoff: false,
      useContext: false
    });
    this.appHelper.pauseMACEComms = true;
  }

  handleCloseDialogs = () => {
    this.setState({
      showEditVehicleHomeDialog: false,
      useContext: false,
      showEditGlobalHomeDialog: false,
      showMessagesMenu: false,
      showConfigDialog: false,
      showTakeoffDialog: false,
      showEnvironmentSettings: false
    });
    this.appHelper.pauseMACEComms = false;
  }

  handleOpenEnvironmentSettings = () => {
    this.setState({showEnvironmentSettings: true});
    this.appHelper.pauseMACEComms = true;
  }

  render() {

    const width = window.screen.width;
    const height = window.screen.height;
    const parentStyle = {height: height + 'px', width: width + 'px'};

    const ToolbarRight = () => (
      <FlatButton
        label={"Sync all"}
        labelPosition={"before"}
        onClick={this.appHelper.handleSyncAll}
        icon={<i className="material-icons">cached</i>}
        style={{color: "white"}}
        />
    );

    return (
        <MuiThemeProvider muiTheme={lightMuiTheme}>
          <div style={parentStyle}>

            <AppBar
                title="MACE"
                style={{backgroundColor: colors.orange700, position: 'fixed'}}
                onLeftIconButtonClick={() => this.setState({openDrawer: !this.state.openDrawer})}
                iconElementRight={<ToolbarRight />}
            />

            <AppDrawer
              openDrawer={this.state.openDrawer}
              onToggleDrawer={(open: boolean) => this.setState({openDrawer: open})}
              onDrawerAction={(action: string) => this.handleDrawerAction(action)}
              showMessagesMenu={this.state.showMessagesMenu}
             />

            <ConnectedVehiclesContainer
              connectedVehicles={this.state.connectedVehicles}
              onAircraftCommand={this.handleAircraftCommand}
              handleChangeSelectedVehicle={this.handleSelectedAircraftUpdate}
              selectedVehicleID={this.state.selectedVehicleID}
            />

            <VehicleCommandsContainer
              connectedVehicles={this.state.connectedVehicles}
              onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
              onAircraftCommand={this.handleAircraftCommand}
              selectedAircraftID={this.state.selectedVehicleID}
              handleTakeoff={this.handleTakeoff}
            />

            {this.state.showEditVehicleHomeDialog &&
              <VehicleHomeDialog
                open={this.state.showEditVehicleHomeDialog}
                handleClose={this.handleCloseDialogs}
                vehicles={this.state.connectedVehicles}
                selectedVehicleID={this.state.selectedVehicleID}
                handleSave={this.appHelper.handleSaveVehicleHome}
                contextAnchor={this.state.contextAnchor}
                useContext={this.state.useContext}
                allowVehicleSelect={this.appHelper.allowVehicleSelect}
                onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                showNotification={this.showNotification}
              />
            }

            {this.state.showEditGlobalHomeDialog &&
              <GlobalOriginDialog
                open={this.state.showEditGlobalHomeDialog}
                handleClose={this.handleCloseDialogs}
                onGlobalHomeCommand={this.handleAircraftCommand}
                globalOrigin={this.appHelper.globalOrigin}
                handleSave={this.appHelper.handleSaveGlobalOrigin}
                contextAnchor={this.state.contextAnchor}
                useContext={this.state.useContext}
              />
            }

            {this.state.showMessagesMenu &&
              <MessagesDialog
                open={this.state.showMessagesMenu}
                handleClose={this.handleCloseDialogs}
                handleSave={this.appHelper.handleSaveMessagingPreferences}
                preferences={this.appHelper.messagePreferences}
              />
            }

            {this.state.showConfigDialog &&
              <ConfigDialog
                open={this.state.showConfigDialog}
                handleClose={this.handleCloseDialogs}
                handleSave={(configSettings: ConfigSettingsType, reload: boolean) => this.appHelper.handleSaveMACEConfig(configSettings, reload)}
                configSettings={this.appHelper.MACEconfig}
                handleParseJSON={(filename: string, restartServer: boolean) => this.appHelper.parseJSONConfig(filename, restartServer)}
              />
            }

            {this.state.showTakeoffDialog &&
              <TakeoffDialog
                open={this.state.showTakeoffDialog}
                handleClose={this.handleCloseDialogs}
                vehicles={this.state.connectedVehicles}
                selectedVehicleID={this.state.selectedVehicleID}
                handleTakeoff={this.appHelper.handleTakeoff}
                takeoffAlt={this.appHelper.MACEconfig.config.VehicleSettings.defaultTakeoffAlt.toString()}
                onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                showSaveTakeoff={this.state.showSaveTakeoff}
                handleSaveTakeoff={(alt: string) => this.handleSaveTakeoff(alt)}
                contextAnchor={this.state.contextAnchor}
                useContext={this.state.useContext}
                showNotification={this.showNotification}
              />
            }


            {this.appHelper.showDraw &&
              <DrawButtonsContainer
                onDeleteLastPolygonPt={this.appHelper.handleDeleteLastPolygonPt}
                onDisableDraw={this.appHelper.handleDisableDraw}
                onSubmitBoundary={this.appHelper.handleSubmitBoundary}
                onClearAllPts={this.appHelper.handleClearPts}
                handleChangeGridSpacing={this.appHelper.handleChangeGridSpacing}
                openEnvironmentSettings={this.handleOpenEnvironmentSettings}
                environmentSettings={this.appHelper.environmentSettings}
              />
            }

            {this.state.showEnvironmentSettings &&
              <EnvironmentSettings
                open={this.state.showEnvironmentSettings}
                handleClose={this.handleCloseDialogs}
                handleSave={this.appHelper.saveEnvironmentSettings}
                environmentSettings={this.appHelper.environmentSettings}
              />
            }


            <MACEMap
              handleSelectedAircraftUpdate={this.handleSelectedAircraftUpdate}
              setContextAnchor={(anchor: L.LeafletMouseEvent) => this.setState({contextAnchor: anchor})}
              connectedVehicles={this.state.connectedVehicles}
              selectedVehicleID={this.state.selectedVehicleID}
              mapCenter={this.appHelper.MACEconfig.config.GUIInit.mapCenter}
              maxZoom={this.appHelper.MACEconfig.config.GUIInit.maxZoom}
              mapZoom={this.appHelper.MACEconfig.config.GUIInit.mapZoom}
              globalOrigin={this.appHelper.globalOrigin}
              updateMapCenter={this.updateMapCenter}
              contextAnchor={this.state.contextAnchor}
              contextSetGlobal={this.contextSetGlobal}
              contextSetHome={this.contextSetHome}
              contextSetTakeoff={this.contextSetTakeoff}
              contextGoHere={this.contextGoHere}
              MACEConnected={this.appHelper.MACEConnected}
              environmentBoundary={this.appHelper.environmentBoundary}
              drawPolygonPts={this.appHelper.drawPolygonPts}
              onAddPolygonPt={this.appHelper.handleAddPolygonPt}
              environmentSettings={this.appHelper.environmentSettings}
              gridPts={this.appHelper.gridPts}
              envBoundingBox={this.appHelper.envBoundingBox}
             />


            <div>
              <NotificationSystem ref="notificationSystem" />
            </div>

          </div>
        </MuiThemeProvider>
    );
  }
}
