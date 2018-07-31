
% Read the waveform and confirm it visually by plotting it.
% while true    
    % Accept a connection from any machine on port 1234.
    t = tcpip('0.0.0.0', 1234, 'NetworkRole', 'server');

    % Open a connection. This will not return until a connection is received.
    fopen(t);
    while t.BytesAvailable == 0
        pause(0.1);
    end
    data = fread(t, t.BytesAvailable);
    fprintf('%s\n', data)
    
    % Cleanup
    fclose(t);
    delete(t);
    clear t
% end

