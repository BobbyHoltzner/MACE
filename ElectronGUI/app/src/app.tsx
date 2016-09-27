//React libraries
import * as React from 'react';
import {render} from 'react-dom';

//Import Container component
import AppContainer from './containers/AppContainer';

type Props = {
};

type State = {
};

class App extends React.Component<Props, State> {
  render () {
    return (
      <AppContainer />
    );
  }
}

// Render to index.html
render(
  <App />,
  document.getElementById('map-container')
);
