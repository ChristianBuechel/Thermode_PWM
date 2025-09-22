function out = QuerySerialPorts(cmd)
% implemented only for windows

if IsOSX
    error('OSX not supported')
end

if IsLinux
  pp = dir('/dev/ttyUSB*');
  pp = [ dir('/dev/ttyACM*'); pp];
  pp = [ dir('/dev/ttyS*'); pp];
  ports = cellstr(strvcat(pp.name));
  prefix = '/dev/';
else
% List of all candidates COM1 to COM256
ports      = strtrim(cellstr(num2str((1:256)', 'COM%i')));
prefix     = '';
end
availPorts = {};
allPorts   = {};

% Disable output of IOPort during probe-run:
oldverbosity = IOPort('Verbosity', 0);

% Test each port for existence:
for i = 1:numel(ports)
    % Try to open:
    [h, errmsg] = IOPort('OpenSerialPort', [prefix ports{i}],'BaudRate=115200,DTR=1,RTS=1');

    % Open succeeded?
    if h >= 0
        % Yes, this is an existing and available port. Close it again:
        IOPort('Close', h);
        % Add to list of available and all ports:
        availPorts{end+1} = ports{i}; %#ok<AGROW>
        allPorts{end+1}   = ports{i}; %#ok<AGROW>
    elseif (any(strfind(errmsg, 'already open')) && IsLinux) || (any(strfind(errmsg, 'grant access')) && IsWin)
        % Failed to open port, because it is busy, but port exists. Add
        % to only to list of all ports:
        allPorts{end+1} = ports{i}; %#ok<AGROW>
    end
end
% Restore output of IOPort after probe-run:
IOPort('Verbosity', oldverbosity);

switch upper(cmd)
    case'ALL'
    out = allPorts;
    case {'AVAILABLE','AVAIL'}
    out = availPorts;
    otherwise
    error(sprintf('%s not supported: use ''all'' or ''available'' \n',cmd));
end
