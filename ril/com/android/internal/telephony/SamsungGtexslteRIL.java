/*
 * Copyright (C) 2012-2014 The CyanogenMod Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.android.internal.telephony;

import static com.android.internal.telephony.RILConstants.*;

import android.content.Context;
import android.media.AudioManager;
import android.os.AsyncResult;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.os.Parcel;
import android.telephony.SmsMessage;
import android.os.SystemProperties;
import android.os.SystemClock;
import android.provider.Settings;
import android.text.TextUtils;
import android.telephony.Rlog;

import android.telephony.SignalStrength;

import android.telephony.PhoneNumberUtils;
import com.android.internal.telephony.RILConstants;
import com.android.internal.telephony.gsm.SmsBroadcastConfigInfo;
import com.android.internal.telephony.cdma.CdmaInformationRecords;
import com.android.internal.telephony.cdma.CdmaInformationRecords.CdmaSignalInfoRec;
import com.android.internal.telephony.cdma.SignalToneUtil;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;

import com.android.internal.telephony.uicc.IccCardApplicationStatus;
import com.android.internal.telephony.uicc.IccCardStatus;

public class SamsungGtexslteRIL extends SamsungSPRDRIL implements CommandsInterface {


    public SamsungGtexslteRIL(Context context, int preferredNetworkType,
            int cdmaSubscription, Integer instanceId) {
        super(context, preferredNetworkType, cdmaSubscription, instanceId);
        mQANElements = SystemProperties.getInt("ro.ril.telephony.mqanelements", 4);
    }

    public SamsungGtexslteRIL(Context context, int networkMode,
            int cdmaSubscription) {
        super(context, networkMode, cdmaSubscription);
        mQANElements = SystemProperties.getInt("ro.ril.telephony.mqanelements", 4);
    }

     @Override
    public void
    setNetworkSelectionModeAutomatic(Message response) {
        // Do nothing :D, coz if you do it, modem will crash :o
    }

    @Override
    public void
    sendUSSD (String ussdString, Message response) {
        RILRequest rr
                = RILRequest.obtain(RIL_REQUEST_SEND_USSD, response);

        byte[] ussdByte = null;
        try {
            ussdByte = GsmAlphabet.stringToGsm8BitPacked(ussdString);
        } catch(Exception e) {
            if (RILJ_LOGD) riljLog("Exception e = " + e);
        }
        String sendData = IccUtils.bytesToHexString(ussdByte);
        if (RILJ_LOGD) riljLog("USSD sendData = " + sendData);
        rr.mParcel.writeString(sendData);
        send(rr);
    }

    @Override
    protected void
    processUnsolicited (Parcel p) {
        Object ret;
        int dataPosition = p.dataPosition(); // save off position within the Parcel
        int response = p.readInt();
        Rlog.e(RILJ_LOG_TAG, "ProcessUnsolicited: " + response);
        switch(response) {
            case RIL_UNSOL_ON_USSD: ret =  responseUSSDStrings(p);
                String[] resp = (String[])ret;

                if (resp.length < 2) {
                    resp = new String[2];
                    resp[0] = ((String[])ret)[0];
                    resp[1] = null;
                }
                if (RILJ_LOGD) unsljLogMore(response, resp[0]);
                if (mUSSDRegistrant != null) {
                    mUSSDRegistrant.notifyRegistrant(
                        new AsyncResult (null, resp, null));
                }
            break;
            case 11008: // RIL_UNSOL_DEVICE_READY_NOTI
                ret = responseVoid(p); // Currently we'll bypass logcat for this first
                break;
            // SAMSUNG STATES
            case 11010: // RIL_UNSOL_AM:
                ret = responseString(p);
                String amString = (String) ret;
                Rlog.d(RILJ_LOG_TAG, "Executing AM: " + amString);

                try {
                    Runtime.getRuntime().exec("am " + amString);
                } catch (IOException e) {
                    e.printStackTrace();
                    Rlog.e(RILJ_LOG_TAG, "am " + amString + " could not be executed.");
                }
                break;
            case 11021: // RIL_UNSOL_RESPONSE_HANDOVER:
                ret = responseVoid(p);
                break;
            case 1036:
                ret = responseVoid(p);
                break;
            default:
                // Rewind the Parcel
                p.setDataPosition(dataPosition);

                // Forward responses that we are not overriding to the super class
                super.processUnsolicited(p);
                return;
        }

    }


    private Object responseUSSDStrings(Parcel p) {
        Rlog.d(RILJ_LOG_TAG, "UNSOL_ON_USSD!");
        String[] response = p.readStringArray();
        if(response.length > 0x2) {
            int num = Integer.parseInt(response[0x2]);
            if(num == 0xf) {
                byte[] dataUssd = IccUtils.hexStringToBytes(response[0x1]);
                response[0x1] = GsmAlphabet.gsm8BitUnpackedToString(dataUssd, 0x0, dataUssd.length);
                return response;
            }
            if(num == 0x48) {
                byte[] bytes = new byte[(response[0x1].length() / 0x2)];
                for(int i = 0x0; i < response[0x1].length(); i = i + 0x2) {
                    bytes[(i / 0x2)] = (byte)Integer.parseInt(response[0x1].substring(i, (i + 0x2)), 0x10);
                }
                try {
                    String utfString = new String(bytes, "UTF-16");
                    response[0x1] = utfString;
                    return response;
                } catch(Exception localException1) {
                }
            }
        }
        return response;
    }

    @Override
    public void
    acceptCall (Message result) {
       RILRequest rr
               = RILRequest.obtain(RIL_REQUEST_ANSWER, result);

       rr.mParcel.writeInt(1);
       rr.mParcel.writeInt(0);

       if (RILJ_LOGD) riljLog(rr.serialString() + "> " + requestToString(rr.mRequest));

       send(rr);
    }
}
