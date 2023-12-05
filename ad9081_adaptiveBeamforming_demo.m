% This demo is modified from:
% https://www.mathworks.com/help/phased/ug/conventional-and-adaptive-beamformers.html
% Created by Jon Kraft (Analog Devices), George Mencoff, and Honglei Chen (Mathworks)
% Setup:
% 	1. Connect 4 Log Periodic antennas to the AD9081 ADCs
% 	2. Connect a stubby (Pluto) antenna to DAC0 -- that will be the signal of interest
% 	3. Connect another stubby antenna to DAC1 - that will be the jammer
% 		a. You can use longer SMA cables (like 3 ft) for the signal and the
% 		jammer antennas
%   4. Run calibration first, and add those into calAnt2, calAnt3, calAnt4

clear all; close all;
set(0,'DefaultFigureWindowStyle','docked')
%set(0,'DefaultFigureWindowStyle','normal')
warning("off", 'MATLAB:system:ObsoleteSystemObjectMixin');

%% Setup
fs = 250e6;  % AD9081 output data rate
SignalFreq = 2800e6;
% Run "ad9081_dig_beamformer_cal" file first, then put in cal values here:
calAnt2 = -94;
calAnt3 = 47;
calAnt4 = -150;
[rx,tx] = setupAD9081(fs,SignalFreq,0.5,0.5);

%% Setup Receive Array
steeringAngles = -90:90;
steeringDirections = [steeringAngles;zeros(size(steeringAngles))];
d = 0.053;  % spacing between antennas

% Array model
array = phased.ULA(NumElements=4,ElementSpacing=d);

%% Collect data from the AD9081 Receive Array
for i=1:100
    collectedData = collectHardwareData(rx,tx,calAnt2,calAnt3,calAnt4,0.5,0.5);
    backgroundData = collectHardwareData(rx,tx,calAnt2,calAnt3,calAnt4,0.0,0.5);
    signalData  = collectHardwareData(rx,tx,calAnt2,calAnt3,calAnt4,0.5,0.0);
    
    % Phase shift Beamformer
    % This is a conventional, phase shifting, beamformer implementation
    if i==1
        psbeamformer = phased.PhaseShiftBeamformer('SensorArray',array,...
            'OperatingFrequency',SignalFreq,'Direction',steeringDirections,...
            'WeightsOutputPort',true);
    end
    [psOut, wPS] = psbeamformer(collectedData);
    
    % Plot the amplitude response of the phase shift beamformer 
    peak_sum_ps = calcPeakGain(psOut);
    if i==1
        plotBeamsteerData(steeringAngles,peak_sum_ps,"PS vs MVDR");
    end

    % MVDR Adaptive Beamformer (with Training Data)
    
    % Find direction of the desired signal and jammers
    % This is only necessary for our demo setup).  
    % Normally we know exactly where we want to point the beam!
    [psDesired, wDesired] = psbeamformer(signalData);
    peak_desired = calcPeakGain(psDesired);
    [gainDesired, indexDesired] = max(peak_desired);
    [psJammer, wJammer] = psbeamformer(backgroundData);
    peak_jammer = calcPeakGain(psJammer);
    [gainJammer, indexJammer] = max(peak_jammer);
    
    desiredAngle = steeringDirections(1,indexDesired);
    jammerAngle = steeringDirections(1,indexJammer);
    
    % Now apply the MVDR adaptive algorithim to the collectedData (signal+jammer)
    if i==1
        mvdrbeamformer = phased.MVDRBeamformer('SensorArray',array,...
            'Direction',steeringDirections,'OperatingFrequency',SignalFreq,...
            WeightsOutputPort=true);
        mvdrbeamformer.TrainingInputPort=true;
    end
    [mvdrTrain, wTrain] = mvdrbeamformer(collectedData, backgroundData);
    
    % Compare the beam weights from conventional (phase shifter) and adaptive (MVDR)
    % The MVDR should automatically find the jammer and place null(s) there
    psPattern = pattern(array,SignalFreq,steeringAngles,0,...
        'Weights',wPS(:,indexDesired),'Type','powerdb',...
        'PropagationSpeed',physconst('LightSpeed'),'Normalize',true,...
        'CoordinateSystem','rectangular');
    psPattern = psPattern';
    mvdrPattern = pattern(array,SignalFreq,steeringAngles,0,...
        'Weights',wTrain(:,indexDesired),'Type','powerdb',...
        'PropagationSpeed',physconst('LightSpeed'),'Normalize',true,...
        'CoordinateSystem','rectangular');
    mvdrPattern = mvdrPattern';
    if i==1
        ax2 = axes(figure);
        plot1 = plot(steeringAngles, psPattern);
        hold on;
        plot2 = plot(steeringAngles, mvdrPattern);
        xlim([-90 90]);
        ylim([-60 0]);
        plot1.XDataSource = 'steeringAngles';
        plot1.YDataSource = 'psPattern';
        plot2.XDataSource = 'steeringAngles';
        plot2.YDataSource = 'mvdrPattern';
        title(ax2,"\color{blue}Conventional \color{black}vs \color{red}MVDR \color{black}Beamforming");
        xlabel(ax2,"Angle (deg)");
        ylabel(ax2,"Amplitude (dBFS)")
    else
        markDesired.reset;
        markJammer.reset;
    end
    markDesired = xline(desiredAngle, '--g', 'Desired Signal');
    markJammer = xline(jammerAngle, '--r', 'Jammer');
    refreshdata
    drawnow
end

%% Helper Functions

function [rx,tx] = setupAD9081(fs,SignalFreq,txGain1,txGain2)
    % Setup AD9081 and collect  data
    uri = 'ip:192.168.0.10';
    
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

    % Tx set up
    tx.EnabledChannels = 1;
    tx.DataSource = 'DMA';
    tx.NCOEnables = [1,1,0,0];
    OffsetFreq = 10e6;
    NCOFreq = SignalFreq-OffsetFreq;
    tx.MainNCOFrequencies = [NCOFreq,NCOFreq,NCOFreq,NCOFreq];
    tx.ChannelNCOGainScales = [txGain1,txGain2,0,0];
    amplitude = 2^15; 
    frequency = OffsetFreq;
    swv1 = dsp.SineWave(amplitude, frequency);
    swv1.ComplexOutput = true;
    swv1.SamplesPerFrame = 2^12;
    swv1.SampleRate = fs;
    y = swv1();
    tx.EnableCyclicBuffers = 1;
    tx(y);
    pause(1);
    
    % Rx Setup
    rx.EnabledChannels = [1,2,3,4];
    rx.MainNCOFrequencies = [SignalFreq,SignalFreq,SignalFreq,SignalFreq];
end

function data = collectHardwareData(rx,tx,calAnt2,calAnt3,calAnt4,txGain1,txGain2)
    tx.ChannelNCOGainScales = [txGain1,txGain2,0,0];
    % Rx Data Gather
    for k=1:10
        valid = false;
        while ~valid
            [out, valid] = rx();
        end
    end

    % Phase Shift the Data using calibration values and rearrange
    data = applyCalibration(out, calAnt2, calAnt3, calAnt4);
end

function data = applyCalibration(out,calAnt2,calAnt3,calAnt4)
    % Map the ADCs to the linear array of 4 antenna elements:
    %     Ant1=ADC2, Ant2=ADC0, Ant3=ADC1, Ant4=ADC3
    ant1 = out(:,3);
    ant2 = out(:,1) * exp(1i*deg2rad(calAnt2));
    ant3 = out(:,2) * exp(1i*deg2rad(calAnt3));
    ant4 = out(:,4) * exp(1i*deg2rad(calAnt4));
    data = [ant1,ant2,ant3,ant4];
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

function peak_sum = calcPeakGain(data)
    peak_sum = [];
    for i = 1:length(data(1,:))
        dBFS_sum = calcDBFS(data(:,i));
        peak_sum(i) = max(dBFS_sum);
    end
end

function plotBeamsteerData(angles,data,plotTitle)
    ax = axes(figure);
    title(ax,plotTitle);
    xlabel(ax,"Angle (deg)");
    ylabel(ax,"Amplitude (dBFS)")
    plot(ax,angles,data);
end


