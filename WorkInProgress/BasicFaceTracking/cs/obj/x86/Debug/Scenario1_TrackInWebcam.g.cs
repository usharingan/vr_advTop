﻿#pragma checksum "C:\Users\Oana\Dropbox\uni\Master\2017SS\VR_Advanced_Topics\Windows-universal-samples-master - Copy\Samples\BasicFaceTracking\shared\Scenario1_TrackInWebcam.xaml" "{406ea660-64cf-4c82-b6f0-42d48172a799}" "BA05B41BB7B86EAC0666AC645D6C7775"
//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace SDKTemplate
{
    partial class TrackFacesInWebcam : 
        global::Windows.UI.Xaml.Controls.Page, 
        global::Windows.UI.Xaml.Markup.IComponentConnector,
        global::Windows.UI.Xaml.Markup.IComponentConnector2
    {
        /// <summary>
        /// Connect()
        /// </summary>
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Microsoft.Windows.UI.Xaml.Build.Tasks"," 14.0.0.0")]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        public void Connect(int connectionId, object target)
        {
            switch(connectionId)
            {
            case 1:
                {
                    this.StatusBlock = (global::Windows.UI.Xaml.Controls.TextBlock)(target);
                }
                break;
            case 2:
                {
                    this.CamPreview = (global::Windows.UI.Xaml.Controls.CaptureElement)(target);
                }
                break;
            case 3:
                {
                    this.VisualizationCanvas = (global::Windows.UI.Xaml.Controls.Canvas)(target);
                }
                break;
            case 4:
                {
                    this.CameraStreamingButton = (global::Windows.UI.Xaml.Controls.Button)(target);
                    #line 42 "..\..\..\Scenario1_TrackInWebcam.xaml"
                    ((global::Windows.UI.Xaml.Controls.Button)this.CameraStreamingButton).Click += this.CameraStreamingButton_Click;
                    #line default
                }
                break;
            case 5:
                {
                    this.CameraSnapshotButton = (global::Windows.UI.Xaml.Controls.Button)(target);
                    #line 43 "..\..\..\Scenario1_TrackInWebcam.xaml"
                    ((global::Windows.UI.Xaml.Controls.Button)this.CameraSnapshotButton).Click += this.CameraSnapshotButton_Click;
                    #line default
                }
                break;
            default:
                break;
            }
            this._contentLoaded = true;
        }

        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Microsoft.Windows.UI.Xaml.Build.Tasks"," 14.0.0.0")]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        public global::Windows.UI.Xaml.Markup.IComponentConnector GetBindingConnector(int connectionId, object target)
        {
            global::Windows.UI.Xaml.Markup.IComponentConnector returnValue = null;
            return returnValue;
        }
    }
}

