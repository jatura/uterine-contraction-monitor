samplefreq = 240; %data sampling rate 240 per sec
s1=zeros(1,5520);
s3=zeros(1,5520);
for j = 1:5520
    s1(j) = dataS1(j);
    s3(j) = dataS3(j);
end
w = 240;        % window size = 500 samples
w2 = 120;       % overlap by half window
nfft = 512;     % 512-fft -> DC + 256 harmonics (positive freq. only)
[s, freq, t] = spectrogram(s1, w, w2, nfft, samplefreq, 'psd');
figure(3)
spectrogram(s1,[],[],[],240,'yaxis')
% ylim([0 1])
[s6, freq6, t6] = spectrogram(s3, w, w2, nfft, samplefreq, 'psd');
figure(4)
spectrogram(s3,[],[],[],240,'yaxis')
% ylim([0 1])
% xlim([2 23])
%s2 = s.*conj(s);
s2 = log(s.*conj(s)+1);     % image enhancement for better visualization
% figure(1)
% plot(s)
% figure(2)
% plot(s2);
% figure(3)
% imshow(s2'/max(max(s2)));   % normalized grayscale image
% xlabel('Freq -- '+string(freq(2)-freq(1))+' Hz/pixel');
% ylabel('Time -- '+string(t(2)-t(1))+' s/pixel');
% colormap(hot);  % set colormap to HOT
