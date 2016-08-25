%% 
%> @brief 
%>
%> @author Shu Wang
%> @param filename
%> 
%
%

close all ;
clear all ;

global Gps GpsTx

tic

Gps.Rf.rtlsdr_id        = '0' ;
Gps.Rf.L1Freq_Hz        = 1575.42e6 ;
Gps.Rf.DcFreq_Hz        = 0 ;
Gps.Rf.CenterFreq_Hz    = Gps.Rf.L1Freq_Hz - Gps.Rf.DcFreq_Hz ;
Gps.Rf.EnableTunerAGC   = false;
Gps.Rf.TunerGain_dB     = 48 ;
Gps.Rf.L1Bandwidth_Hz   = 1.023e6 ;
Gps.Rf.OversampleRate   = 1 ;
Gps.Rf.SampleFreq_Hz    = 2.728e6 ;
Gps.Rf.SamplesPerFrame  = 256 * 1023 ;  %Expected SamplesPerFrame to be a scalar with value <= 375000.
Gps.Rf.OutputDataType   = 'single' ;
Gps.Rf.rtlsdr_ppm       = 0 ;

if( Gps.Rf.EnableTunerAGC )
    
    Gps.Rf.radio = comm.SDRRTLReceiver( Gps.Rf.rtlsdr_id, ...
                                'CenterFrequency', Gps.Rf.CenterFreq_Hz, ...
                                'EnableTunerAGC', Gps.Rf.EnableTunerAGC , ...
                                'SampleRate', Gps.Rf.SampleFreq_Hz, ...
                                'SamplesPerFrame', Gps.Rf.SamplesPerFrame, ...
                                'OutputDataType', Gps.Rf.OutputDataType, ...
                                'FrequencyCorrection', Gps.Rf.rtlsdr_ppm ) ;
                            
else
    
    Gps.Rf.radio = comm.SDRRTLReceiver( Gps.Rf.rtlsdr_id, ...
                                'CenterFrequency', Gps.Rf.CenterFreq_Hz, ...
                                'EnableTunerAGC', Gps.Rf.EnableTunerAGC , ...
                                'TunerGain', Gps.Rf.TunerGain_dB, ...
                                'SampleRate', Gps.Rf.SampleFreq_Hz, ...
                                'SamplesPerFrame', Gps.Rf.SamplesPerFrame, ...
                                'OutputDataType', Gps.Rf.OutputDataType, ...
                                'FrequencyCorrection', Gps.Rf.rtlsdr_ppm ) ;
    
end

Gps.Rf.sdrHwInfo = info( Gps.Rf.radio ) ;
assert( false == isLocked( Gps.Rf.radio ), [ Gps.Rf.sdrHwInfo.RadioName '@' Gps.Rf.sdrHwInfo.RadioAddress ' is not locked' ] ) ;               


Gps.Analyzer.rfSpectrum = dsp.SpectrumAnalyzer( ...
    'Name', 'Spectrum Analyzer FFT', ...
    'Title', 'Spectrum Analyzer FFT', ...
    'SpectrumType', 'Power density', ...
    'FrequencySpan', 'Start and stop frequencies', ...
    'StartFrequency',   - Gps.Rf.SampleFreq_Hz / 2, ...
    'StopFrequency',    + Gps.Rf.SampleFreq_Hz / 2, ...
    'SampleRate', Gps.Rf.SampleFreq_Hz ) ;

toc

Gps.Rf.initalizationTime_s = 600 ;
Gps.Rf.stopTime_s          = Gps.Rf.SamplesPerFrame / Gps.Rf.SampleFreq_Hz * 8 
Gps.Rf.radioFrameTime_s    = Gps.Rf.SamplesPerFrame / Gps.Rf.SampleFreq_Hz ;

%% Stream Processing Loop
%
% Capture GPS signals for 10 seconds which is specified by Gps.Rf.StopTime_ms.

%%
% Check for the status of the RTL-SDR radio

if ~isempty( sdrinfo( Gps.Rf.radio.RadioAddress ) )

    % Loop until the example reaches the simulation stop time
    timeCounter = 0;
    while timeCounter < Gps.Rf.initalizationTime_s
        % Get baseband samples from RTL-SDR radio
        [rfBuff, ~] = step( Gps.Rf.radio );  % no 'len' output needed for blocking operation
        
        step( Gps.Analyzer.rfSpectrum, rfBuff )  ;
        
        % Update counter
        timeCounter = timeCounter + Gps.Rf.radioFrameTime_s ;
    end
    
    toc
    
    % Loop until the example reaches the simulation stop time
    timeCounter = 0 ;
    frameCount  = 0 ;
    
    while timeCounter < Gps.Rf.stopTime_s
        % Get baseband samples from RTL-SDR radio
        [ rfBuff, ~ ] = step( Gps.Rf.radio );  % no 'len' output needed for blocking operation
        
        assert( length( rfBuff ) == Gps.Rf.SamplesPerFrame, 'Not enough RF samples come out' ) ;

        Gps.Rf.cData( (1:Gps.Rf.SamplesPerFrame) + Gps.Rf.SamplesPerFrame *  frameCount, 1 ) = rfBuff - mean( rfBuff );
        
        % Update counter
        timeCounter = timeCounter + Gps.Rf.radioFrameTime_s ;
        frameCount = frameCount + 1 ;
    end
    
    %% 
    % Release the audio and RTL-SDR resources.
    release( Gps.Rf.radio ) ;
    
    Gps.timeStamp = num2str( round( now * 1e4 ) ) ; 

    Gps.Rf.rfDataLen   = length( Gps.Rf.cData ) ;
    Gps.Rf.iqData      = resample( double([real( Gps.Rf.cData ) imag( Gps.Rf.cData )]), 3, 2 ) ;
    Gps.Rf.iqDataLen   = length( Gps.Rf.iqData ) ;

    dlmwrite( [ Gps.timeStamp '_' Gps.Rf.sdrHwInfo.TunerName '.rfd' ], Gps.Rf.cData ) ;
    dlmwrite( [ Gps.timeStamp '_' Gps.Rf.sdrHwInfo.TunerName '.inq' ], Gps.Rf.iqData, '\t' ) ;
    
    toc
    
    Gps.Rf.ADC.removeDC      = true ;
    Gps.Rf.ADC.fixed         = false ;
    Gps.Rf.ADC.magCount      = [0 0] ;
    Gps.Rf.ADC.threshold     = [0 0] ;

    %%
    %> Check the quality of the received RF signals
    %>
    Gps.Rf.signal.mean   = [0 0] ;
    Gps.Rf.signal.std    = [0 0] ;

    Gps.Rf.signal.mean = mean( Gps.Rf.iqData ) ;

    %%
    %> remove DC if there is ANY. Most likely this is unecessary.
    %>
    if( Gps.Rf.ADC.removeDC )
        Gps.Rf.iqData(:, 1) = Gps.Rf.iqData(:, 1) - Gps.Rf.signal.mean(1) ;
        Gps.Rf.iqData(:, 2) = Gps.Rf.iqData(:, 2) - Gps.Rf.signal.mean(2) ;
        disp( [' Rf.ADC.removeDC == ' Gps.Rf.ADC.removeDC ] ) ;
    end

    %%
    %>  check the standard deviation for checkig signal quality and setting 
    %> the ADC threshold if necessary. 
    %
    Gps.Rf.signal.std = std( Gps.Rf.iqData ) ;

    %%
    %>  The ADC quantizes IF or ZF signals into 2-bit real digitalized IF 
    %> or ZF data comprising MAG and SIGN components. The MAG values control 
    %> the AGC loop, such that the MAG bit is active (HIGH) for approximately 
    %> 33% of the time.
    %>
    %>  1-Sigma deviation is about 34.1 percent point of the normal 
    %> distribution.
    %>

    if( false == Gps.Rf.ADC.fixed )      
        Gps.Rf.ADC.threshold = Gps.Rf.signal.std 
    end

    Gps.Rf.ADC.threshold

    %%
    %>  GPS RF signals will be digitalized and compressed into bytes: One I
    %> or Q sample being compressed into two bits and two RF samples being 
    %> compressed into one byte.
    %
    toc
    for ii = 2:2:Gps.Rf.iqDataLen

        %% 
        %>  Convert and compress on two consecutive complex sample data into 
        %> one byte.
        %> 

        %%
        %> The expected ADC output is 2 bits, [ -3 -1 +1 +3 ] ;
        %>
        %> in = 0x01 = +1 --> ot = 0b0000 0000 = +0
        %> in = 0x03 = +3 --> ot = 0x0000 0010 = +2
        %> in = 0xFF = -1 --> ot = 0x0000 0001 = +1
        %> in = 0xFD = -3 --> ot = 0x0000 0011 = +3
        %>
        %> The percentage of the 2s and 3s in ADC outptus should be roughly 33
        %> percent.
        %>
        for jj = (ii-1):1:ii

            for IQ = 1:2

                %% ADC
                if( Gps.Rf.iqData( jj, IQ ) >= +Gps.Rf.ADC.threshold( IQ ) )

                    Gps.Rf.adcData( jj*2-2 + IQ ) = int8( +3 ) ;

                    Gps.Rf.piqBuff( jj, IQ ) = uint8( 2 ) ;

                    Gps.Rf.ADC.magCount( IQ ) = Gps.Rf.ADC.magCount( IQ ) + 1 ;

                elseif( Gps.Rf.iqData( jj, IQ ) >= 0 ) 

                    Gps.Rf.adcData( jj*2-2 + IQ ) = int8( +1 ) ;

                    Gps.Rf.piqBuff( jj, IQ ) = uint8( 0 ) ;

                elseif( Gps.Rf.iqData( jj, IQ ) >= -Gps.Rf.ADC.threshold( IQ ) )

                    Gps.Rf.adcData( jj*2-2 + IQ ) = int8( -1 ) ;

                    Gps.Rf.piqBuff( jj, IQ ) = uint8( 1 ) ;

                else

                    Gps.Rf.adcData( jj*2-2 + IQ ) = int8( -3 ) ;

                    Gps.Rf.piqBuff( jj, IQ ) = uint8( 3 ) ;

                    Gps.Rf.ADC.magCount( IQ ) = Gps.Rf.ADC.magCount( IQ ) + 1 ;

                end

            end

            %% Compress the first complex sample into a 4-bit or half-byte value
    %         Gps.Rf.piqData( jj ) = uint8( bitxor( bitshift( Gps.Rf.adcData(jj, 1), 2, 'uint8' ), Gps.Rf.adcData(jj, 2) ) ) ;
    %         Gps.Rf.piqData( jj ) = uint8( bitxor( bitshift( Gps.Rf.piqData( jj ), 4, 'uint8' ), Gps.Rf.piqData( jj ) ) ) ;   

            Gps.Rf.piqData( jj ) = uint8( ( Gps.Rf.piqBuff(jj, 1) * 4 + Gps.Rf.piqBuff(jj, 2) ) * 17 ) ;

        end

    end

    toc

    Gps.Rf.piqFilename   = [ Gps.timeStamp '_' Gps.Rf.sdrHwInfo.TunerName '.piq' ] ;
    Gps.Rf.permission    = 'a' ;
    Gps.Rf.machinefmt    = 'n' ;
    Gps.Rf.piqFid        = fopen( Gps.Rf.piqFilename, Gps.Rf.permission, Gps.Rf.machinefmt ) ;   

    fwrite( Gps.Rf.piqFid, Gps.Rf.piqData, 'uint8' ) ;
    fclose( Gps.Rf.piqFid ) ;

    Gps.Rf.adcFilename   = [ Gps.timeStamp '_' Gps.Rf.sdrHwInfo.TunerName '.bin' ] ;
    Gps.Rf.permission    = 'a' ;
    Gps.Rf.machinefmt    = 'n' ;
    Gps.Rf.adcFid        = fopen( Gps.Rf.adcFilename, Gps.Rf.permission, Gps.Rf.machinefmt ) ;   

    fwrite( Gps.Rf.adcFid, Gps.Rf.adcData, 'int8' ) ;
    fclose( Gps.Rf.adcFid ) ;
    
else
    
    [ status, cmdout ]                        = unix( 'ls -1 -t *.bin', '-echo' ) ;
    [ Gps.Data.fileList, Gps.Data.fileCount ] = textscan(cmdout, '%s') ;
    
    Gps.Rf.adcFilename  = char( Gps.Data.fileList{1,1}(1,1) )
    
end

toc

Gps.Search.freqBias_Hz    = Gps.Rf.DcFreq_Hz ;
Gps.Search.coherentMode   = 18 ;
Gps.Search.nonCoherentCnt = 40 ;
Gps.Search.coherentIntvl  = 20 ;
Gps.Search.window_bin     = 250 ;
% Gps.Search.assistFilename = [Gps.filename '.asi'] ;

%%
%> run the native Gps.Rf GPS Searcher program.
%> 
%>  For a full search without assist, the GPS Searcher needs only the data
%> file as its input. 
%
command = ['./GpsSearcher2 ' Gps.Rf.adcFilename ...
            ' --fo ' int2str( Gps.Search.freqBias_Hz ) ...
            ' --ct ' int2str( Gps.Search.coherentMode ) ...
            ' --ci ' int2str( Gps.Search.coherentIntvl ) ...
            ' -w ' int2str( Gps.Search.window_bin  ) ] 

%% Have memory issues? 
% Uncommon the instruction below to clean out more memory.
% clear ComTech Gps.Rf

switch computer
    
    case 'PCWIN64'
        [status, cmdout] = dos( command, '-echo' ) ;        
    
    otherwise
        [status, cmdout] = unix( command, '-echo' ) ;
        
end

%save( [Gps.Rf.filename '.mat'] )
