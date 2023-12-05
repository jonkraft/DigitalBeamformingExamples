clear all; close all;
warning("off", 'MATLAB:system:ObsoleteSystemObjectMixin');

txSource = 'DMA'; % DMA or DDS
uri = 'ip:192.168.0.10';
fs = 250e6;  % this is not changeable -- set with device tree on the SD card
SignalFreq = 2800e6;
OffsetFreq = 10e6;
NCOFreq = SignalFreq-OffsetFreq;
d = 0.053;  % spacing between antennas in m (d=3e8/NCOFreq /2);

% Run "ad9081_dig_beamformer_cal" file first, then put in cal values here:
calAnt2 = -94;
calAnt3 = 57;
calAnt4 = -159;

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
tx.MainNCOFrequencies = [NCOFreq,NCOFreq,NCOFreq,NCOFreq];
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

%% Phase Shift the Data
decimationRatio = 16;
fs_dec=fs/decimationRatio;
delay_phases = (-180:2:180);
steer_angles = calc_theta(delay_phases, SignalFreq, d);
[delayed_sum, peak_sum] = formBeam(out, decimationRatio, delay_phases, calAnt2, calAnt3, calAnt4);

%% Plot
plotFFT = false;
if plotFFT == true  
    df = fs_dec/length(delayed_sum);
    freqRangeRx = (-fs_dec/2:df:fs_dec/2-df).'/1e6;
    FFTPlot = plot(freqRangeRx, delayed_sum);
    xlabel('Frequency (MHz)');ylabel('Amplitude (dBFS)');grid on;
    FFTPlot.XDataSource = 'freqRangeRx';
    FFTPlot.YDataSource = 'delayed_sum';
else
    steerPlot = plot(steer_angles, peak_sum);
    title("4 Channel Digital Beam Array Pattern")
    xlabel('Steering Angle (deg)');ylabel('Amplitude (dBFS)');grid on;
    xlim([-90 90]); xticks(-90:15:90);
    ylim([-70 -20]);
    steerPlot.XDataSource = 'steer_angles';
    steerPlot.YDataSource = 'peak_sum';
end

for i=1:5
    [out, valid] = rx();
    [delayed_sum, peak_sum] = formBeam(out, decimationRatio, delay_phases, calAnt2, calAnt3, calAnt4);
    refreshdata
    drawnow
    pause(0.1);
end

rx.release();
tx.release();
beep on
beep

%% FUNctions

function [delayed_sum, peak_sum] = formBeam(data, dec, delays, calAnt2, calAnt3, calAnt4)
    % Map the ADCs to the linear array of 4 antenna elements:
    %     Ant1=ADC2, Ant2=ADC0, Ant3=ADC1, Ant5=ADC3
    Ant1 = decimate(data(:,3), dec);
    Ant2 = decimate(data(:,1), dec);
    Ant3 = decimate(data(:,2), dec);
    Ant4 = decimate(data(:,4), dec);
    
    peak_sum = [];
    for i = 1:length(delays)
        phase_delay = delays(:,i);
        delayed_Ant2 = phase_delayer(Ant2, phase_delay+calAnt2);
        delayed_Ant3 = phase_delayer(Ant3, phase_delay*2+calAnt3);
        delayed_Ant4 = phase_delayer(Ant4, phase_delay*3+calAnt4);
        delayed_sum = calcDBFS(Ant1 + delayed_Ant2 + delayed_Ant3 + delayed_Ant4);
        peak_sum(i) = max(delayed_sum);
    end
end

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
