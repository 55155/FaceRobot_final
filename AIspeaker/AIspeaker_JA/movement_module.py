import os
from config import CURRENT_PATH


def generation_head_mouth_motion(audioname):
    currentpath = CURRENT_PATH
    audiofilepath = currentpath+'Audio/'+audioname+'.wav'
    headcsvfilepath = currentpath+'Headmotion/'+audioname+'.csv'
    mouthcsvfilepath = currentpath+'Mouthmotion/'+audioname+'-delta-big.csv'
    # audiofilepath = audioname+'.wav'
    # headcsvfilepath = audioname+'.csv'
    # mouthcsvfilepath = audioname+'-delta-big.csv'
    segmentfolder = currentpath+'segments/'

    ####################################################################################################################################
    #################################################################################################################################### MOUTH
    ####################################################################################################################################

    import scipy.io.wavfile as wav
    import numpy as np
    from scipy import interpolate
    from scipy.signal import butter, filtfilt
    from scipy.signal import find_peaks

    max_MOUTH = 0 #default_MOUTH
    min_MOUTH = 350 # 250
    ratio_minOPEN = 0.5

    # Envelope
    peak_distance = 1000 

    # Low pass filter
    num_div = 7 #Larger, smaller fc
    shift = 3

    # max_sigma
    num = 0.8

    # thresholdnum
    thresholdnum = 0.1

    ####################################################################################################################################
    ####################################################################################################################################
    ####################################################################################################################################

    "## ========================= Audio Read"
    ## https://stackoverflow.com/questions/39316087/how-to-read-a-audio-file-in-python-similar-to-matlab-audioread
    ## https://kr.mathworks.com/help/matlab/ref/audioread.html?lang=en

    fs,y = wav.read(audiofilepath)

    N = y.shape[0]   # number of samples
    Time_sec = (N-1)/fs  # total time (sec)

    y = y / np.iinfo(y.dtype).max  # y range -1 ~ +1

    # xtime = np.linspace(0,Time,N,endpoint = True)
    xtime = np.arange(N)*1/fs + 0

    # ## Plot soundwave
    # plt.figure(0,figsize=(30,5)); plt.grid(True)
    # plt.plot(xtime,y)
    # plt.title('Original soundwave')
    # plt.xlabel('Time (sec)')

    if y.ndim != 1:
        y = y[:, 0]  # use channel 1

    "## ======================= Envelope"
    ## Envelope and interpolate
    ## https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.find_peaks.html
    ## https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html

    interpolate_kind = 'cubic' # 'quadratic' 'linear'

    ## Find high envelope index
    yy = y - np.mean(y)
    high_idx,_ = find_peaks(yy, distance=peak_distance)

    ## Pad with initial and final index
    if high_idx[0] != 0:
        high_idx = np.pad(high_idx,(1,0),'constant',constant_values=0)
    if high_idx[-1] != yy.shape[0]-1:
        high_idx = np.pad(high_idx,(0,1),'constant',constant_values=yy.shape[0]-1)  

    ## Interpolate the local max. points
    f = interpolate.interp1d(xtime[high_idx], yy[high_idx],kind=interpolate_kind) # 'quadratic' 'cubic'
    up = f(xtime)

    "## ======================= Sampling and Simple Smoothing"

    sampling_interval_ms = 40 # ms

    ## Sampling
    N_size = int(np.floor(Time_sec*1000 / sampling_interval_ms))

    up_sampled = np.zeros(N_size)
    y_sampled = np.zeros(N_size)

    sampling_interval_index = int(np.floor((N-1)/(N_size-1)))
    for i in range(0,N_size):
        up_sampled[i] = up[i*sampling_interval_index]
        y_sampled[i] = y[i*sampling_interval_index]

    ## Simple Smoothing (no negative values)
    soundwave_threshold_zero = 0.05 * np.max(np.abs(y_sampled))
    for i in range(0,y_sampled.shape[0]):
        if abs(y_sampled[i]) < 0.0001:
            up_sampled[i] = 0

        if up_sampled[i] < 0 and abs(y_sampled[i]) > soundwave_threshold_zero:
            up_sampled[i] *= -1
        elif up_sampled[i] < 0:
            up_sampled[i] = 0    

    # t2 = np.arange(0,y_sampled.shape[0])
    # t2 = t2 * sampling_interval_ms / 1000

    # plt.figure(1,figsize=(30,5)); plt.grid(True)
    # plt.plot(t2,y_sampled)
    # plt.plot(t2,up_sampled,'r')
    # plt.title('Sampled y and upper envelope')

    "## ======================= Envelope Smooting"

    ## Remove too large values

    nonzeros_upnew = up_sampled.copy()
    iter = 0
    for i in range(0,up_sampled.shape[0]):
        if abs(y_sampled[i]) > soundwave_threshold_zero:
            if up_sampled[i] != 0:
                nonzeros_upnew[iter] = up_sampled[i]
                iter += 1
    nonzeros_upnew = nonzeros_upnew[0:iter]

    max_Upnew = np.mean(nonzeros_upnew) + num * np.std(nonzeros_upnew)
    up_sampled_maxcut = up_sampled.copy()
    for i in range(0,up_sampled_maxcut.shape[0]):
        if up_sampled_maxcut[i] > max_Upnew:
            up_sampled_maxcut[i] = max_Upnew

    # plt.figure(2,figsize=(30,5)); plt.grid(True)
    # plt.plot(t2,y_sampled)
    # plt.plot(t2,up_sampled_maxcut,'r')
    # plt.title('Maximum value smoothed upper envelope')

    "## ======================= Low Pass Filter"
    ## Define low pass filter
    ## https://stackoverflow.com/questions/25191620/creating-lowpass-filter-in-scipy-understanding-methods-and-units
    ## https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.lfilter.html
    ## https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html

    def butter_lowpass(cutoff, fs, order=5):
        return butter(order, cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(data, cutoff, fs, order=5):
        b, a = butter_lowpass(cutoff, fs, order=order)
        y = filtfilt(b, a, data)
        return y

    ## Low pass filter
    fc = round(fs / num_div)
    up_sampled_LPF = butter_lowpass_filter(up_sampled_maxcut, fc, fs, order=3)

    ## Compensates for the delay introduced by the filter with shifting
    #up_sampled_LPF = np.zeros(up_sampled_LPF_raw.shape[0])
    #up_sampled_LPF[0:-shift] = up_sampled_LPF_raw[shift:up_sampled_LPF_raw.shape[0]]
    for i in range(0,shift):
        up_sampled_LPF[i-shift] = up_sampled_LPF[-1] 

    ## Noise reduction
    threshold = thresholdnum * np.max(up_sampled_LPF)
    for i in range(0,up_sampled_LPF.shape[0]):
        if up_sampled_LPF[i] < threshold:
            up_sampled_LPF[i] = 0

    # plt.figure(3,figsize=(50,8)); plt.grid(True)
    # plt.plot(t2,y_sampled)
    # plt.plot(t2,up_sampled_LPF,'r')
    # plt.title('Smoothed upper envelope (low pass filter fc={} Hz)'.format(fc))

    "## ======================= min_OPEN"
    mouth_raw = up_sampled_LPF.copy()

    min_OPEN = ratio_minOPEN*max(up_sampled_LPF)

    lmin = (np.diff(np.sign(np.diff(up_sampled_LPF))) > 0).nonzero()[0] + 1
    smallpeaks,_ = find_peaks(up_sampled_LPF,height =(None,min_OPEN), prominence = 0.15*max(up_sampled_LPF))

    for i in range(0,smallpeaks.shape[0]):
        lminidx = np.searchsorted(lmin, smallpeaks[i])
        prevMin = lmin[lminidx-1]
        laterMin = lmin[lminidx]

        prev = np.ceil((2*smallpeaks[i]+prevMin)/3)
        later = np.floor((2*smallpeaks[i]+laterMin)/3)
        xx = np.array([prevMin,prev,smallpeaks[i],later,laterMin])

        y = np.array([up_sampled_LPF[prevMin], (5*min_OPEN + up_sampled_LPF[prevMin]) / 6,
                      min_OPEN, (5*min_OPEN + up_sampled_LPF[laterMin]) / 6,up_sampled_LPF[laterMin]])


        try:
            f = interpolate.interp1d(xx, y,kind='quadratic') # 'quadratic' 'cubic'
        except:
            j=1
            while(prev == smallpeaks[i]):
                if up_sampled_LPF[prevMin-1] == 0:
                    prevMin -= 1
                else:
                    prevMin = lmin[lminidx-1-j]
                prev = np.ceil((2*smallpeaks[i]+prevMin)/3)
                j+=1

            k=1
            while(later == smallpeaks[i]):
                if up_sampled_LPF[laterMin+1] == 0:
                    laterMin += 1 
                else:
                    laterMin = lmin[lminidx+k]
                later = np.floor((2*smallpeaks[i]+laterMin)/3)
                k+=1     

            xx = np.array([prevMin,prev,smallpeaks[i],later,laterMin])

            y = np.array([up_sampled_LPF[prevMin], (5*min_OPEN + up_sampled_LPF[prevMin]) / 6,
                          min_OPEN, (5*min_OPEN + up_sampled_LPF[laterMin]) / 6,up_sampled_LPF[laterMin]])

            f = interpolate.interp1d(xx, y,kind='quadratic')

        realtime_x = np.arange(prevMin, laterMin)
        for i in range(0,realtime_x.shape[0]):
            if up_sampled_LPF[realtime_x[i]]<f(realtime_x[i]):
                up_sampled_LPF[realtime_x[i]] = f(realtime_x[i])


    # plt.figure(4,figsize=(100,4)); plt.grid(True);
    # plt.plot(t2,mouth_raw,label='mouth_raw')
    # plt.plot(t2,up_sampled_LPF,'r',label='min_OPEN')
    # plt.title('After Applying Min_open')
    # plt.legend(loc='lower right')

    "## ======================= Antiq-Sound-Close"
    ## For natural mouth opening (antiq) and closing
    ## https://scienceon.kisti.re.kr/commons/util/originalView.do?cn=JAKO201820540193966&oCn=JAKO201820540193966&dbt=JAKO&journal=NJOU00560240

    ASC = up_sampled_LPF.copy()

    # -0.4: mouth open
    # -0.3: between close~open
    # -0.2: mouth close

    flag_hold_zero = True # hold zero value
    for i in range(0,ASC.shape[0]-1):

        ## When positive sound decreases and becomes zero
        if not flag_hold_zero and up_sampled_LPF[i] == 0:
            ASC[i] = -0.2 # mouth close
            flag_hold_zero = True

        ## When zero sound is about to increase to positive
        if flag_hold_zero and up_sampled_LPF[i] == 0 and up_sampled_LPF[i+1] != 0: 
            if ASC[i] == -0.2: # if mouth close was just signaled from above [if statement] (point zero)
                ASC[i] = -0.3  # then do something between close~open
            else:              # if mouth is currently closed (2 or more consecutive zeros)
                ASC[i] = -0.4  # then open mouth
            flag_hold_zero = False

        ## When there exists positive sound
        if up_sampled_LPF[i] != 0:
            flag_hold_zero = False

    # plt.figure(5,figsize=(30,5)); plt.grid(True)
    # plt.plot(t2,y_sampled,label='voice')
    # plt.plot(t2,up_sampled_LPF,'r',label='upper envelope')
    # plt.stem(t2,ASC,markerfmt='*',linefmt=':',label='ASC')
    # plt.title('Before Applying Antiq-Sound-Close')
    # plt.legend(loc='lower right')

    "## ======================= Smooth Antiq, Close"

    ## Smooth closing
    up2mouth = up_sampled_LPF.copy()
    L = up2mouth.shape[0]

    Close = np.zeros(6)

    # ASC[0] == -0.2 -> never happens
    # ASC[1] == -0.2 -> no treatment required

    # ASC[2]~ASC[L-7] == -0.2
    # Mouth close signaled at i
    # Smooth from i-1 to i+4 using value at i-2 (this should be positive)
    for i in range(2,L-6): # -3
        if ASC[i] == -0.2:
            for c in range(0,6): # (0,4)
                Close[c] = up_sampled_LPF[i-2]*(6-c)/7 # (4-c)/5
            for c in range(0,6): # (0,4)
                if Close[c] > up_sampled_LPF[i+c-1]:
                    up2mouth[i+c-1] = Close[c]

    # ASC[L-6]~ASC[L-1] == -0.2
    for i in range(L-6,L): # -3
        if ASC[i] == -0.2:
            for c in range(0,L-i):
                Close[c] = up_sampled_LPF[i-2]*(L-i-c)/(L-i+1)
            for c in range(0,L-i):
                if Close[c] > up_sampled_LPF[i+c-1]:
                    up2mouth[i+c-1] = Close[c]        

    ## Smooth opening
    Antiq = np.zeros(3)

    # ASC[0] == -0.4 -> no treatment required
    # ASC[1] == -0.4: ASC[0:3] 0 0 x -> 0 x/2 x
    if ASC[1] == -0.4:
        up2mouth[1] = up_sampled_LPF[2]/2

    # ASC[2]~ASC[L-2] == -0.4
    # Mouth open signaled at i
    # Smooth from i-2 to i using value at i+1 (this should be positive)
    for i in range(2,L-1):
        if ASC[i] == -0.4:
            for j in range(0,3):
                Antiq[j] = up_sampled_LPF[i+1]*j/3
            for j in range(0,3):
                if Antiq[j] > up2mouth[i-2+j]:
                    up2mouth[i-2+j] = Antiq[j]

    # ASC[L-1] == -0.4 -> never happens

    ## Btw. close~open
    # ASC[0] or ASC[L-1] == -0.3 -> never happens
    for i in range(1,L-1):
        if ASC[i] == -0.3:
            up2mouth[i] = (up2mouth[i-1] + up2mouth[i+1])/3

    # plt.figure(6,figsize=(90,5)); plt.grid(True)
    # plt.plot(t2,y_sampled,label='voice')
    # plt.plot(t2,up_sampled_LPF,'m',label='upper envelope')
    # plt.plot(t2,up2mouth,'r',label='envelope to mouth')
    # plt.title('After Applying Antiq-Sound-Close')
    # plt.legend(loc='lower right')


    "## ======================= Scaling to DXL_MOUTH"

    max_up_new = np.max(up2mouth)
    mouth = max_MOUTH*np.ones(up2mouth.shape[0]) - up2mouth*(max_MOUTH-min_MOUTH)/max_up_new


    # plt.figure(7,figsize=(30,5)); plt.grid(True);
    # # plt.plot(t2,mouth_raw,label='mouth_raw')
    # plt.plot(t2,mouth,'r',label='Mouth Motor Input')
    # plt.title('Mouth Motor Input')
    # plt.legend(loc='lower right')


    "## ======================= head_gesture_ratio"
    # Keep head softly when robot is not singing
    # data : mouth, ratio : ratio of head gesture you want when robot is not singing//0~1

    def find_zero(data, ratio):
        start_rest_idx = []
        rest_len = []
        ratio_remainder = int(10-10*ratio)
        if ratio < 0.5:
            min_len = 2 * ratio_remainder
        else:
            min_len = 10

        count = 1
        head_gesture_ratio = np.ones(N_size)
        # print(head_gesture_ratio)
        head_gesture_ratio_idx = (np.where(data == max_MOUTH)[0])
        for i in range(head_gesture_ratio_idx.shape[0]-1):
            if head_gesture_ratio_idx[i + 1] == head_gesture_ratio_idx[i]+1:
                count += 1
            else:
                start_rest_idx.append(head_gesture_ratio_idx[i] - count + 1)
                rest_len.append(count)
                count = 1
        # print(start_rest_idx)
        # print(rest_len)
        for i in range(len(rest_len)):
            if rest_len[i] >= min_len:
                for j in range(ratio_remainder):
                    head_gesture_ratio[start_rest_idx[i]+j] = 0.9 -j *0.1
                if rest_len[i] > 2*ratio_remainder:
                    for j in range(ratio_remainder,rest_len[i]-ratio_remainder):
                        head_gesture_ratio[start_rest_idx[i]+j] = ratio
                k = 0
                for j in range(rest_len[i]-ratio_remainder,rest_len[i]):
                    head_gesture_ratio[start_rest_idx[i]+j] = ratio + k *0.1
                    k += 1
        return head_gesture_ratio

    head_gesture_ratio = find_zero(mouth,0.6)
    # print(head_gesture_ratio[500:1000])

    ####################################################################################################################################
    ####################################################################################################################################
    ####################################################################################################################################

    output_mouth_ratio = np.column_stack((mouth,head_gesture_ratio))
    np.savetxt(mouthcsvfilepath,output_mouth_ratio, fmt='%.10e', delimiter=',')
    #print('---------------------- NEW MOUTH MOTION GENERATED')

    ####################################################################################################################################
    #################################################################################################################################### HEAD
    ####################################################################################################################################

    import numpy as np
    import math
    import pickle
    import librosa
    from scipy import interpolate
    import random

    dt = 40
    segmentLength = 360          # ms
    shift = 80                   # ms (overlap = segmentLength - shift)
    L = round(segmentLength/dt)  # length
    l = round(shift/dt)          # shift

    classNum = 4

    increaseAmp = 1.5
    rpymax = [0.7,0.6,1.0]

    audiofeaturewindow = 40 # ms
    audiofeatureoverlap = 23.3 # ms

    rpy_grad = 'one2one'
    audio_grad = 'one2one'

    ################################################################################################################################# BASIC

    def makeXtime(Time_ms,feature):

        if type(feature) == int:
            return np.round(np.linspace(0, Time_ms, num=feature, endpoint=True)) # ms
        elif type(feature) == np.ndarray:
            return np.round(np.linspace(0, Time_ms, num=feature.shape[0], endpoint=True)) # ms

    ################################################################################################################################# AUDIO FEATURES

    def tmp_syncAudioVideoOneFeature(xtime_audiofeature,xtime_video,audiofeature1,delta=15):

        audiofeature_sync = np.zeros(xtime_video.shape[0])

        for i in range(audiofeature_sync.shape[0]):
            for j in range(xtime_audiofeature.shape[0]):
                flag_found = False
                if abs(xtime_video[i]-xtime_audiofeature[j]) < delta:
                    audiofeature_sync[i] = audiofeature1[j]
                    flag_found = True
                    break
            if not flag_found:
                print("error")

        return audiofeature_sync   

    def syncAudioVideo(xtime_audiofeature,xtime_video,audiofeature,delta=15):

        if audiofeature.ndim == 1:
            return tmp_syncAudioVideoOneFeature(xtime_audiofeature,xtime_video,audiofeature,delta)
        else:
            audiofeature_sync = np.zeros([xtime_video.shape[0],audiofeature.shape[1]])
            for i in range(audiofeature.shape[1]):
                audiofeature_sync[:,i] = tmp_syncAudioVideoOneFeature(xtime_audiofeature,xtime_video,audiofeature[:,i],delta)#.reshape(-1,1)

            return audiofeature_sync     

    def MeanFiltering(original,filtersize=1,shift=0):    

        Time = original.shape[0]

        filtered = original.copy()

        if original.ndim != 1:

            iteration = 0
            for i in range(Time):
                if iteration < filtersize-1:
                    # filtered[i,:] = np.round(np.mean(original[i-iteration:i+1,:],axis=0)) # mean of rows (column direction)
                    filtered[i,:] = np.mean(original[i-iteration:i+1,:],axis=0) # mean of rows (column direction)
                else:
                    # filtered[i,:] = np.round(np.mean(original[i-(filtersize-1):i+1,:],axis=0))
                    filtered[i,:] = np.mean(original[i-(filtersize-1):i+1,:],axis=0)

                iteration = iteration + 1

        else:

            iteration = 0
            for i in range(Time):
                if iteration < filtersize-1:                
                    filtered[i] = np.mean(original[i-iteration:i+1],axis=0) # mean of rows (column direction)
                else:
                    # filtered[i,:] = np.round(np.mean(original[i-(filtersize-1):i+1,:],axis=0))
                    filtered[i] = np.mean(original[i-(filtersize-1):i+1],axis=0)

                iteration = iteration + 1

        filtered[0:Time-shift] = filtered[shift:]

        return filtered

    def makeNoVoiceSteps(voiced_flag_sync):
        novoicesteps = np.zeros(voiced_flag_sync.shape[0])

        novoice = 0
        for i in range(novoicesteps.shape[0]):
            if voiced_flag_sync[i] == 0:
                novoice += 1
            elif voiced_flag_sync[i] == 1:
                novoice = 0

            novoicesteps[i] = novoice

        return novoicesteps

    def makeNoVoiceExp(energy,thr=0.01,div=10):

        voiceflag_e = energy.copy()
        voiceflag_e[voiceflag_e< thr]=0
        voiceflag_e[voiceflag_e>=thr]=1
        novoicesteps = makeNoVoiceSteps(voiceflag_e)
        novoiceexp = np.exp(-novoicesteps/div)
        novoiceexp[novoiceexp<=0.3]=0.3

        # novoiceexp = np.r_[ novoiceexp , np.array([novoiceexp[-1]]*100) ]

        return novoiceexp,voiceflag_e

    ################################################################################################################################# 

    def getSegmentAverageGrad(y,delta='one2one',mode='abs'):

        x = y.copy()
        if x.ndim == 1:
            x = x.reshape(-1,1)

        if delta == 'one2one':
            grad = x[1:,:]-x[:-1,:] # T-1 x dim
        elif delta == 'end2end':
            grad = x[-1,:]-x[0,:] # dim
        else:
            print('getSegmentAverageGrad delta error')

        if mode == 'abs':
            grad = np.abs(grad)
        elif mode == 'pos':
            grad = grad[grad>0]
            # grad[grad<0] = 0
        elif mode == 'neg':
            grad = grad[grad<0]
            # grad[grad>0] = 0
        elif mode == 'org':
            pass
        else:
            print('getSegmentAverageGrad mode error')

        if grad.size==0:
            return 0

        return np.average(grad)

    def assignClassWith1DMiddleBoundary(x,boundaries):
        # boundaries: no min,max; only middle boundaries; ex) ~1,1~2,2~3,3~ -> 1,2,3
        for i in range(len(boundaries)-1):
            if x < boundaries[i]:
                return i
            elif x < boundaries[i+1]:
                return i+1
        return len(boundaries)

    ################################################################################################################################# HEAD MOTION GENERATION

    def getNextSegment_PointSeg(PrevEnd,rpysegmentsClass,gradient=True):

        # PrevEnd: 3(rpy)
        # rpysegmentsClass: L x 3(rpy) x numberofsegments

        distselectnum = 20
        distselectdist = 0.2
        randomchoosenum = 5 # < distselectnum
        # randomchoosenum = 1 # < distselectnum

        distselectnum = min(distselectnum,rpysegmentsClass.shape[2]) # rpysegmentsClass.shape[2] = len(dists)
        randidx = random.randint(0,randomchoosenum-1)

        startpoints = rpysegmentsClass[0,:,:].T
        endpoint_tile = np.tile(PrevEnd.reshape(1,-1),[rpysegmentsClass.shape[2],1])

        dists = np.linalg.norm(endpoint_tile - startpoints,axis=1)

        ## dists[minsortedindex[0]] : minimum distance
        ## rpysegmentsClass[:,:,minsortedindex[0]] : segment with minimum distance     
        distminsortedindex = np.array( sorted(range(len(dists)), key=lambda k: dists[k]) )[:distselectnum]
        distusingindices = distminsortedindex[ dists[distminsortedindex] < distselectdist ] # dists[distusingindices]: 최소 거리 순

        if len(distusingindices) < randomchoosenum:
            index = np.argmin(dists)
            return rpysegmentsClass[:,:,index]

        # min gradient
        if gradient:  
            firsttwopoints = rpysegmentsClass[:2,:,distusingindices]      # shape: 2 x 3(rpy) x distselectnum; 거리 짧은 순으로 계산됨
            startgrads = firsttwopoints[1,:,:] - firsttwopoints[0,:,:]    # shape: 3(rpy) x distselectnum
            gradnorms = np.linalg.norm(startgrads,axis=0)                 # shape: distselectnum
            gradminsortedindex = sorted(range(len(gradnorms)), key=lambda k: gradnorms[k])
            index = distusingindices[gradminsortedindex[randidx]]        
        else:
            index = distusingindices[randidx]          

        return rpysegmentsClass[:,:,index]

    def getNextSegment_SegSeg(PrevEndOneBefore,PrevEnd,rpysegmentsClass,gradient=True,gotozero=True):

        # PrevEndOneBefore: 3(rpy)
        # PrevEnd: 3(rpy)
        # rpysegmentsClass: L x 3(rpy) x numberofsegments

        distselectnum = 20 
        distselectdist = 0.2
        gradselectnum = 15 # < distselectnum
        randomchoosenum = 10 # < distselectnum,gradselectnum
        # randomchoosenum = 1 # < distselectnum,gradselectnum

        distselectnum = min(distselectnum,rpysegmentsClass.shape[2]) # rpysegmentsClass.shape[2] = len(dists)
        randidx = random.randint(0,randomchoosenum-1) # 0 ~ randomchoosenum-1

        startpoints = rpysegmentsClass[0,:,:].T
        endpoint_tile = np.tile(PrevEnd.reshape(1,-1),[rpysegmentsClass.shape[2],1])

        dists = np.linalg.norm(endpoint_tile - startpoints,axis=1)

        ## dists[minsortedindex[0]] : minimum distance
        ## rpysegmentsClass[:,:,minsortedindex[0]] : segment with minimum distance     
        distminsortedindex = np.array( sorted(range(len(dists)), key=lambda k: dists[k]) )[:distselectnum]        
        distusingindices = distminsortedindex[ dists[distminsortedindex] < distselectdist ]

        if len(distusingindices) < randomchoosenum:
            index = np.argmin(dists)
            return rpysegmentsClass[:,:,index]

        # min gradient difference
        if gradient:

            gradselectnum = min(gradselectnum,len(distusingindices))

            firsttwopoints = rpysegmentsClass[:2,:,distusingindices]             # shape: 2 x 3(rpy) x distselectnum
            startgrads = firsttwopoints[1,:,:] - firsttwopoints[0,:,:]       # shape: 3(rpy) x distselectnum

            gradfinal = PrevEnd - PrevEndOneBefore
            gradfinal_tile = np.tile(gradfinal.reshape(-1,1),[1,len(distusingindices)]) # shape: 3(rpy) x distselectnum

            graddists = np.linalg.norm(gradfinal_tile - startgrads,axis=0)   # shape: distselectnum            
            gradminsortedindex = sorted(range(len(graddists)), key=lambda k: graddists[k])[:gradselectnum]
            gradusingindices = np.array(distusingindices)[gradminsortedindex] # dist 작은 것들 grad 작은 순으로 정렬한 index

            # go to zero - weighted scores
            if gotozero:

                tmp_segments = rpysegmentsClass[:,:,gradusingindices] # dist, grad 조건 충족한 segment들 (일정 dist 이하들을 grad 작은 순으로 정렬); L x 3 x gradselectnum            
                tmp_rpyscores = np.multiply(tmp_segments[0,:,:],tmp_segments[-1,:,:] - tmp_segments[0,:,:]) # shape 3 x gradselectnum            
                gotozeroscores = np.sum(tmp_rpyscores,axis=0) # shape gradselectnum            

                scoreminsortedindex = sorted(range(len(gotozeroscores)), key=lambda k: gotozeroscores[k])

                # mm+randidx < gradselectnum-1
                mm = 0
                index = gradusingindices[scoreminsortedindex[mm+randidx]]            
            else:
                index = gradusingindices[randidx]
        else:
            index = distusingindices[randidx]    

        return rpysegmentsClass[:,:,index]

    def ConnectPointAndSegment(point,test2,n_interpolate=5,n_new=4):

        # 1 from point, n_interpolate from test2 -> 1 + n_interpolate used for interpolation
        # n_interpolate must be >= 3
        # n_new new points

        point = point.copy()
        test2 = test2.copy()

        nn = n_interpolate + n_new    

        if nn > test2.shape[0]:
            print('point-segment interpolation warning')
            k = 0
            while nn > test2.shape[0]:
                if k%2 == 0 and n_new > 1:
                    n_new -= 1                
                elif k%2 == 1 and n_interpolate >= 4:                
                    n_interpolate -= 1                
                nn = n_interpolate + n_new                
                k += 1
                if k > 200:
                    print('point-segment interpolation failed')
                    return np.append(point.reshape(1,-1),test2,axis=0)

        x = list(range(nn+1))

        for rpy in range(3):
            y = [point[rpy]] + test2[:nn,rpy].tolist()

            x_interpolate = [x[0]] + x[-n_interpolate:]
            y_interpolate = [y[0]] + y[-n_interpolate:]
            tck = interpolate.splrep(x_interpolate, y_interpolate)

            y_new = [y[0]]
            for i in range(1,1+n_new):
                y_new += [interpolate.splev(i, tck)]
            y_new += y[-n_interpolate:]

            test2[:nn,rpy] = np.array(y_new[-nn:])

        return np.append(point.reshape(1,-1),test2,axis=0)

    def ConnectTwoSegments(test1,test2,n_interpolate=3,n_new=4):

        # n_interpolate from test1, n_interpolate from test2 -> 2*n_interpolate used for interpolation
        # n_interpolate must be >= 2
        # 2*n_new new points

        test1 = test1.copy()
        test2 = test2.copy()

        nn = n_interpolate + n_new

        if nn > test2.shape[0]:
            print('segment-segment interpolation warning')
            k = 0
            while nn > test2.shape[0]:
                if k%2 == 0 and n_new > 1:
                    n_new -= 1
                elif k%2 == 1 and n_interpolate >= 3:                
                    n_interpolate -= 1                
                nn = n_interpolate + n_new  
                k += 1
                if k > 100:
                    print('segment-segment interpolation failed')
                    return np.append(test1,test2,axis=0)

        x = list(range(nn*2))

        for rpy in range(3):
            y = test1[-nn:,rpy].tolist() + test2[:nn,rpy].tolist()

            x_interpolate = x[0:n_interpolate] + x[-n_interpolate:]        
            y_interpolate = y[0:n_interpolate] + y[-n_interpolate:]
            tck = interpolate.splrep(x_interpolate, y_interpolate)

            y_new = y[0:n_interpolate]
            for i in range(n_interpolate,n_interpolate+2*n_new):
                y_new += [interpolate.splev(i, tck)]
            y_new += y[-n_interpolate:]

            test1[-nn:,rpy] = np.array(y_new[:nn])
            test2[:nn,rpy] = np.array(y_new[-nn:])

        return np.append(test1,test2,axis=0)

    def multExpToSegment(novoiceexp_seg,NextSegment):

        delta = NextSegment[1:,:] - NextSegment[:-1,:]
        newNextSegment = NextSegment.copy()

        for i in range(1,newNextSegment.shape[0]):
            # newNextSegment[i,:] = newNextSegment[i-1,:] + novoiceexp_seg[i-1] * delta[i-1,:] # onset delayed one step
            newNextSegment[i,:] = newNextSegment[i-1,:] + novoiceexp_seg[i] * delta[i-1,:] # onset not delayed

        return newNextSegment

    def wav2rpy(wavpath,audiofeaturewindow,audiofeatureoverlap,rpysegments_parallel,e_bd,div=10):

        y, fs = librosa.load(wavpath, sr=None)    
        Time_ms = 1000*(y.shape[0]-1)/fs 

        # print('Length of audio file = {:.2f} seconds'.format(Time_ms/1000))

        energy = librosa.feature.rms(y=y , frame_length=round(audiofeaturewindow/1000*fs) , hop_length=round(audiofeatureoverlap/1000*fs) )[0]
        energy = MeanFiltering(energy,5,2) # mean filtering

        xtime_rpy = makeXtime(Time_ms,math.floor(Time_ms/dt))
        xtime_audio = makeXtime(Time_ms,energy)

        energy = syncAudioVideo(xtime_audio,xtime_rpy,energy,delta=13)

        L = rpysegments_parallel[0].shape[0]

        novoiceexp = makeNoVoiceExp(energy,thr=0.01,div=div)[0]
        novoiceexp = np.r_[ novoiceexp , np.array([novoiceexp[-1]]*100) ]

        rpy_generated = np.empty([0,3])

        i = 0
        e_delta_val_sq = []
        while i+L < energy.shape[0]:

            e_seg = energy[i:i+L]
            e_delta_val = getSegmentAverageGrad(e_seg,delta=audio_grad,mode='abs')

            e_delta_val_sq += [e_delta_val] * e_seg.shape[0]

            segClass = 0 if i==0 else assignClassWith1DMiddleBoundary(e_delta_val,e_bd)

            # print(segClass)

            novoiceexp_seg = novoiceexp[i:i+L]

            if i == 0:            
                NextSegment = getNextSegment_PointSeg( np.zeros(3) , rpysegments_parallel[segClass] , gradient=True )
                NextSegment = multExpToSegment(novoiceexp_seg,NextSegment)
                rpy_generated = ConnectPointAndSegment( np.zeros(3) , NextSegment , n_interpolate=5 , n_new=3 )
            else:
                NextSegment = getNextSegment_SegSeg( rpy_generated[-2,:] , rpy_generated[-1,:] , rpysegments_parallel[segClass] , gradient=True , gotozero=True )
                NextSegment = multExpToSegment(novoiceexp_seg,NextSegment)
                rpy_generated = ConnectTwoSegments( rpy_generated , NextSegment , n_interpolate=3 , n_new=4 )            

            i += L

        NextSegment = getNextSegment_SegSeg( rpy_generated[-2,:] , rpy_generated[-1,:] , rpysegments_parallel[3] , gradient=True , gotozero=True )    
        NextSegment = multExpToSegment(novoiceexp[i:i+NextSegment.shape[0]],NextSegment)

        rpy_generated = ConnectTwoSegments( rpy_generated , NextSegment , n_interpolate=3 , n_new=4 )    
        rpy_generated = rpy_generated[:energy.shape[0],:]

        e_delta_val_sq += [0] * (rpy_generated.shape[0] - len(e_delta_val_sq))

        return rpy_generated, xtime_rpy , energy , e_delta_val_sq

    ####################################################################################################################################
    ####################################################################################################################################
    ####################################################################################################################################

    rpysegmentsavepath = segmentfolder+'rpysegments_'+str(segmentLength) + '_' + rpy_grad +   '_'+str(classNum)+      '.npy'
    audioboundsavepath = segmentfolder+'energybd_'+str(segmentLength) + '_' + audio_grad +    '_'+str(classNum)+     '.npy'

    with open(rpysegmentsavepath, 'rb') as f:
        rpysegments_parallel = pickle.load(f)
    with open(audioboundsavepath,'rb') as f:
        e_bd = pickle.load(f)

    ####################################################################################################################################
    ####################################################################################################################################
    ####################################################################################################################################

    rpy_generated = wav2rpy( audiofilepath , audiofeaturewindow , audiofeatureoverlap , rpysegments_parallel , e_bd , div=5 )[0]

    rpy_generated_delta = np.zeros(rpy_generated.shape)
    rpy_generated_delta[1:,:] = rpy_generated[1:,:] - rpy_generated[:-1,:]

    np.savetxt(headcsvfilepath, rpy_generated, delimiter=",",fmt='%f')

    #print('---------------------- NEW HEAD MOTION GENERATED')
    
    return True