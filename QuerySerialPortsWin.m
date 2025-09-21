function ports = QuerySerialPortsWin(cmd)
% implemented only for windows

if IsOSX || IsLinux
    error('OSX and Linux not supported')
end

% List of all candidates COM1 to COM256
ports      = strtrim(cellstr(num2str((1:256)', 'COM%i')));
availPorts = {}; 
allPorts   = {};

% Disable output of IOPort during probe-run:
oldverbosity = IOPort('Verbosity', 0);

% Test each port for existence:
for i = 1:numel(ports)
    % Try to open:
    [h, errmsg] = IOPort('OpenSerialPort', ports{i},'BaudRate=115200,DTR=1,RTS=1');
    
    % Open succeeded?
    if h >= 0
        % Yes, this is an existing and available port. Close it again:
        IOPort('Close', h);        
        % Add to list of available and all ports:
        availPorts{end+1} = ports{i}; %#ok<AGROW>
        allPorts{end+1}   = ports{i}; %#ok<AGROW>
    elseif isempty(strfind(errmsg, 'ENOENT'))
        % Failed to open port, because it is busy, but port exists. Add
        % to only to list of all ports:
        allPorts{end+1} = ports{i}; %#ok<AGROW>
    end
end
% Restore output of IOPort after probe-run:
IOPort('Verbosity', oldverbosity);

switch upper(cmd)
    case'ALL'
    ports = allPorts;
    case {'AVAILABLE','AVAIL'}
    ports = availPorts;    
    otherwise
    error(sprintf('%s not supported: use ''all'' or ''available'' \n',cmd));
end    
