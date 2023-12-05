clear all; close all;
warning("off", 'MATLAB:system:ObsoleteSystemObjectMixin');

fprintf("Running Calibration -- place signal source at mechanical boresight!\n")

txSource = 'DMA'; % DMA or DDS
uri = 'ip:192.168.0.10';
fs = 250e6;  % this is not changeable -- set with device tree on the SD card
SignalFreq = 2800e6;
OffsetFreq = 10e6;
NCOFreq = SignalFreq-OffsetFreq;
d = 0.053;  % spacing between antennas in m (d=3e8/NCOFreq /2);

% Get Device configuration automatically
tx = adi.AD9081.Tx('uri',uri);
[cdc, fdc, dc] = tx.GetDataPathConfiguration();
tx = adi.AD9081.Tx(...
    'uri',uri,...
    'num_data_channels', dc, ...
    'num_coarse_attr_channels', cdc, ...
    'num_fine_attr_channels', fdc);
rx = adi.AD9081.Rx('uri',uri);
[cdc, fdc, dc] = rx.GetDataPathConfiguration();
rx = adi.AD9081.Rx(...
    'uri',uri,...
    'num_data_channels', dc, ...
    'num_coarse_attr_channels', cdc, ...
    'num_fine_attr_channels', fdc);

%% Tx set up
tx.EnabledChannels = 1;
tx.DataSource = txSource;
tx.NCOEnables = [1,0,0,0];
tx.MainNCOFrequencies = [NCOFreq,0,0,0];
txGain = 0.5;  % set between 0 and 0.5
tx.ChannelNCOGainScales = [txGain,0,0,0];
if strcmpi(txSource,'DDS')
    toneFreq = OffsetFreq;
    scales = 0.5;
    phases = 0;
    tx.DDSFrequencies = repmat(toneFreq,2,4);
    tx.DDSScales = repmat(scales,2,4);
    tx.DDSPhases = [0,0;90000,0].';
    tx();
elseif strcmpi(txSource,'DMA')
    amplitude = 2^15; 
    frequency = OffsetFreq;
    swv1 = dsp.SineWave(amplitude, frequency);
    swv1.ComplexOutput = true;
    swv1.SamplesPerFrame = 2^12;
    swv1.SampleRate = fs;
    y = swv1();
    tx.EnableCyclicBuffers = 1;
    tx(y);
else
    error('Invalid value of txSource, must be DMA or DDS');
end
pause(1);

%% Rx Setup
rx.EnabledChannels = [1,2,3,4];
rx.MainNCOFrequencies = [SignalFreq,SignalFreq,SignalFreq,SignalFreq];

%% Rx Data Gather
for k=1:10
    valid = false;
    while ~valid
        [out, valid] = rx();
    end
end
rx.release();
tx.release();

%% Phase Shift the Data
decimationRatio = 16;
fs_dec=fs/decimationRatio;
% Map the ADCs to the linear array of 4 antenna elements:
%     Ant1=ADC2, Ant2=ADC0, Ant3=ADC1, Ant4=ADC3
Ant1 = decimate(out(:,3), decimationRatio);
Ant2 = decimate(out(:,1), decimationRatio);
Ant3 = decimate(out(:,2), decimationRatio);
Ant4 = decimate(out(:,4), decimationRatio);

delay_phases = (-180:.1:180);
peak_sum2 = [];
peak_sum3 = [];
peak_sum4 = [];

for i = 1:length(delay_phases)
    phase_delay = delay_phases(:,i);
    delayed_Ant2 = phase_delayer(Ant2, phase_delay);
    delayed_sum = calcDBFS(Ant1 - delayed_Ant2);
    peak_sum2(i) = max(delayed_sum);
end
[value2, index2] = min(peak_sum2);
calAnt2 = delay_phases(index2);
for i = 1:length(delay_phases)
    phase_delay = delay_phases(:,i);
    delayed_Ant2 = phase_delayer(Ant2, phase_delay+calAnt2);
    delayed_Ant3 = phase_delayer(Ant3, phase_delay*2);
    delayed_sum = calcDBFS(delayed_Ant2 - delayed_Ant3);
    peak_sum3(i) = max(delayed_sum);
end
[value3, index3] = min(peak_sum3);
calAnt3 = delay_phases(index3);
for i = 1:length(delay_phases)
    phase_delay = delay_phases(:,i);
    delayed_Ant3 = phase_delayer(Ant3, phase_delay*2+calAnt3);
    delayed_Ant4 = phase_delayer(Ant4, phase_delay*3);
    delayed_sum = calcDBFS(delayed_Ant3 - delayed_Ant4);
    peak_sum4(i) = max(delayed_sum);
end
[value4, index4] = min(peak_sum4);
calAnt4 = delay_phases(index4);

peak_sum = [];
for i = 1:length(delay_phases)
    phase_delay = delay_phases(:,i);
    delayed_Ant2 = phase_delayer(Ant2, phase_delay+calAnt2);
    delayed_Ant3 = phase_delayer(Ant3, phase_delay*2+calAnt3);
    delayed_Ant4 = phase_delayer(Ant4, phase_delay*3+calAnt4);
    delayed_sum = calcDBFS(Ant1 + delayed_Ant2 + delayed_Ant3 + delayed_Ant4);
    peak_sum(i) = max(delayed_sum);
end

%% Plot
plotFFT = false;
if plotFFT == true  
    df = fs_dec/length(delayed_sum);
    freqRangeRx = (-fs_dec/2:df:fs_dec/2-df).'/1e6;
    plot(freqRangeRx, delayed_sum);
    xlabel('Frequency (MHz)');ylabel('Amplitude (dBFS)');grid on;
else
    [value, index] = min(peak_sum);
    delay_phases(index);
    plot(delay_phases, peak_sum);
    xlabel('Steering Angle (deg)');ylabel('Amplitude (dBFS)');grid on;
    xlim([-180 180]); xticks(-180:15:180);
    ylim([-90 0]);
end

fprintf("calAnt2 = %.1f\n", calAnt2)
fprintf("calAnt3 = %.1f\n", calAnt3)
fprintf("calAnt4 = %.1f\n", calAnt4)


%% FUNctions
function s_dbfs = calcDBFS(data)
    % calculates dBFS value of data for a 12 bit ADC
    nSamp = length(data);
    win = hamming(nSamp);
    y = data .* win;
    s_fft = fft(y)/sum(win);
    s_shift = fftshift(s_fft);
    s_dbfs = 20*log10(abs(s_shift)/(2^11));
end

function delayed_data = phase_delayer(data, delay)
    % phase delay in degrees
    delayed_data = data * exp(i*deg2rad(delay));
end

function theta = calc_theta(delay, freq, d)
    % calculates steering angle for a given phase delay (deg), freq (Hz), 
    % and element spacing d (m)
    arcsin_arg = deg2rad(delay)*3E8/(2*pi*freq*d);
    arcsin_arg = max(min(1, arcsin_arg), -1);
    theta = rad2deg(asin(arcsin_arg));
end
