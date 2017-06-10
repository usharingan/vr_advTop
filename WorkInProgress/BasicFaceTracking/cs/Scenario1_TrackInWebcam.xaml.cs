//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

using Windows.Graphics.Imaging;
using Windows.Media;
using Windows.Media.Capture;
using Windows.Media.FaceAnalysis;
using Windows.Media.MediaProperties;
using Windows.System.Threading;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using Windows.UI.Xaml.Shapes;

using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Storage;
using Windows.Storage.Streams;
using System.IO;
using System.Linq;
using System.Numerics;

using Windows.Devices.Enumeration;
using Windows.UI.Core;

namespace SDKTemplate
{
    /// <summary>
    /// Page for demonstrating FaceTracking.
    /// </summary>
    public sealed partial class TrackFacesInWebcam : Page
    {
        /// <summary>
        /// Brush for drawing the bounding box around each identified face.
        /// </summary>
        private readonly SolidColorBrush lineBrush = new SolidColorBrush(Windows.UI.Colors.Yellow); /*Red*/

        /// <summary>
        /// Thickness of the face bounding box lines.
        /// </summary>
        private readonly double lineThickness = 2.0;

        /// <summary>
        /// Transparent fill for the bounding box.
        /// </summary>
        private readonly SolidColorBrush fillBrush = new SolidColorBrush(Windows.UI.Colors.Transparent);

        /// <summary>
        /// Reference back to the "root" page of the app.
        /// </summary>
        private MainPage rootPage;

        /// <summary>
        /// Holds the current scenario state value.
        /// </summary>
        private ScenarioState currentState;

        /// <summary>
        /// References a MediaCapture instance; is null when not in Streaming state.
        /// </summary>
        private MediaCapture mediaCapture;

        /// <summary>
        /// Cache of properties from the current MediaCapture device which is used for capturing the preview frame.
        /// </summary>
        private VideoEncodingProperties videoProperties;

        /// <summary>
        /// References a FaceTracker instance.
        /// </summary>
        private FaceTracker faceTracker;
        
     // inserted by Oana
        /// <summary>
        /// References a FaceDetector instance.
        /// </summary>
        private FaceDetector faceDetector;

        /// <summary>
        /// A periodic timer to execute FaceTracker on preview frames
        /// </summary>
        private ThreadPoolTimer frameProcessingTimer;

        /// <summary>
        /// Semaphore to ensure FaceTracking logic only executes one at a time
        /// </summary>
        private SemaphoreSlim frameProcessingSemaphore = new SemaphoreSlim(1);

        // test
        private int count = 0;

        //BasicFaceRecognizer model = FaceRecognizer.CreateEigenFaceRecognizer();

        private SemaphoreSlim snapshotSemaphore = new SemaphoreSlim(1);
        // Test
        private SemaphoreSlim writeToFileSemaphore = new SemaphoreSlim(1);

        /*     // TEST camera rotation...
             // a boolean member variable to track whether the camera is external to the device, such as a USB web cam
             private bool _externalCamera;
             // a boolean variable to track whether the preview should be mirrored, which is the case if a front-facing camera is used
             private bool _mirroringPreview;
             // a variable for storing a DeviceInformation object that represents the selected camera
             DeviceInformation _cameraDevice;
             private CameraRotationHelper _rotationHelper;
         */
        IList<DetectedFace> facesPrevFrame;
        int[] faceIDsPrevFrame;
        int[] faceIDsCurrFrame;
        Vector2 faceMovementDirection;
        bool firstFrame;
        int faceCounter;

        /// <summary>
        /// Constructor
        /// Initializes a new instance of the <see cref="TrackFacesInWebcam"/> class.
        /// </summary>
        public TrackFacesInWebcam()
        {

            //test
            facesPrevFrame = null;
            faceIDsCurrFrame = null;
            faceIDsPrevFrame = null;
            faceMovementDirection = new Vector2(0.0f,0.0f);
            firstFrame = true;


            this.InitializeComponent();

            this.currentState = ScenarioState.Idle;
            App.Current.Suspending += this.OnSuspending;
        }

        /// <summary>
        /// Values for identifying and controlling scenario states.
        /// </summary>
        private enum ScenarioState
        {
            /// <summary>
            /// Display is blank - default state.
            /// </summary>
            Idle,

            /// <summary>
            /// Webcam is actively engaged and a live video stream is displayed.
            /// </summary>
            Streaming
        }

        /// <summary>
        /// Responds when we navigate to this page.
        /// </summary>
        /// <param name="e">Event data</param>
        protected override async void OnNavigatedTo(NavigationEventArgs e)
        {
            this.rootPage = MainPage.Current;

            // The 'await' operation can only be used from within an async method but class constructors
            // cannot be labeled as async, and so we'll initialize FaceTracker here.
            if (this.faceTracker == null)
            {
                this.faceTracker = await FaceTracker.CreateAsync();
            }

            // The 'await' operation can only be used from within an async method but class constructors
            // cannot be labeled as async, and so we'll initialize FaceDetector here.
            if (this.faceDetector == null)
            {
                this.faceDetector = await FaceDetector.CreateAsync();
            }
      
        }

        /// <summary>
        /// Responds to App Suspend event to stop/release MediaCapture object if it's running and return to Idle state.
        /// </summary>
        /// <param name="sender">The source of the Suspending event</param>
        /// <param name="e">Event data</param>
        private void OnSuspending(object sender, Windows.ApplicationModel.SuspendingEventArgs e)
        {
            if (this.currentState == ScenarioState.Streaming)
            {
                var deferral = e.SuspendingOperation.GetDeferral();
                try
                {
                    this.ChangeScenarioState(ScenarioState.Idle);
                }
                finally
                {
                    deferral.Complete();
                }
            }
        }

        /// <summary>
        /// Initializes a new MediaCapture instance and starts the Preview streaming to the CamPreview UI element.
        /// </summary>
        /// <returns>Async Task object returning true if initialization and streaming were successful and false if an exception occurred.</returns>
        private async Task<bool> StartWebcamStreaming()
        {
            bool successful = true;

            try
            {
                // TEST
       /*         var allVideoDevices = await DeviceInformation.FindAllAsync(DeviceClass.VideoCapture);
                DeviceInformation desiredDevice = allVideoDevices.FirstOrDefault(x => x.EnclosureLocation != null
                    && x.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Front);
                _cameraDevice = desiredDevice ?? allVideoDevices.FirstOrDefault();
                if (_cameraDevice == null)
                {
                    System.Diagnostics.Debug.WriteLine("No camera device found!");
                  //  return;
                }
                var settings = new MediaCaptureInitializationSettings { VideoDeviceId = _cameraDevice.Id };
        */
                this.mediaCapture = new MediaCapture();

                // For this scenario, we only need Video (not microphone) so specify this in the initializer.
                // NOTE: the appxmanifest only declares "webcam" under capabilities and if this is changed to include
                // microphone (default constructor) you must add "microphone" to the manifest or initialization will fail.
//
                MediaCaptureInitializationSettings settings = new MediaCaptureInitializationSettings();
                settings.StreamingCaptureMode = StreamingCaptureMode.Video;

                await this.mediaCapture.InitializeAsync(settings);
      //          this.mediaCapture.RecordLimitationExceeded += MediaCapture_RecordLimitationExceeded;
                this.mediaCapture.Failed += this.MediaCapture_CameraStreamFailed;
            //  this.mediaCapture.CameraStreamStateChanged += this.MediaCapture_CameraStreamStateChanged;

                // Cache the media properties as we'll need them later.
                var deviceController = this.mediaCapture.VideoDeviceController;
                this.videoProperties = deviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview) as VideoEncodingProperties;

                // Immediately start streaming to our CaptureElement UI.
                // NOTE: CaptureElement's Source must be set before streaming is started.
                this.CamPreview.Source = this.mediaCapture;
                await this.mediaCapture.StartPreviewAsync();

                // TEST
       /*         // Handle camera device location
                if (_cameraDevice.EnclosureLocation == null ||
                    _cameraDevice.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Unknown)
                {
                    _externalCamera = true;
                }
                else
                {
                    _externalCamera = false;
                    _mirroringPreview = (_cameraDevice.EnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Front);
                }
                _rotationHelper = new CameraRotationHelper(_cameraDevice.EnclosureLocation);
                _rotationHelper.OrientationChanged += RotationHelper_OrientationChanged;
        */
                // tracker
                // Use a 66 millisecond interval for our timer, i.e. 15 frames per second
                TimeSpan timerInterval = TimeSpan.FromMilliseconds(66);
                this.frameProcessingTimer = Windows.System.Threading.ThreadPoolTimer.CreatePeriodicTimer(new Windows.System.Threading.TimerElapsedHandler(ProcessCurrentVideoFrame), timerInterval);
            }
            catch (System.UnauthorizedAccessException)
            {
                // If the user has disabled their webcam this exception is thrown; provide a descriptive message to inform the user of this fact.
                this.rootPage.NotifyUser("Webcam is disabled or access to the webcam is disabled for this app.\nEnsure Privacy Settings allow webcam usage.", NotifyType.ErrorMessage);
                successful = false;
            }
            catch (Exception ex)
            {
                this.rootPage.NotifyUser(ex.ToString(), NotifyType.ErrorMessage);
                successful = false;
            }

            return successful;
        }

        /// <summary>
        /// Safely stops webcam streaming (if running) and releases MediaCapture object.
        /// </summary>
        private async void ShutdownWebCam()
        {
        // tracker
            if(this.frameProcessingTimer != null)
            {
                this.frameProcessingTimer.Cancel();
            }

            if (this.mediaCapture != null)
            {
                if (this.mediaCapture.CameraStreamState == Windows.Media.Devices.CameraStreamState.Streaming)
                {
                    try
                    {
                        await this.mediaCapture.StopPreviewAsync();
                    }
                    catch(Exception)
                    {
                        ;   // Since we're going to destroy the MediaCapture object there's nothing to do here
                    }
                }
                this.mediaCapture.Dispose();
            }

            this.frameProcessingTimer = null;
            this.CamPreview.Source = null;
            this.mediaCapture = null;
            this.CameraStreamingButton.IsEnabled = true;

        }

        // tracker
        // should run in front
        /// <summary>
        /// This method is invoked by a ThreadPoolTimer to execute the FaceTracker and Visualization logic at approximately 15 frames per second.
        /// </summary>
        /// <remarks>
        /// Keep in mind this method is called from a Timer and not synchronized with the camera stream. Also, the processing time of FaceTracker
        /// will vary depending on the size of each frame and the number of faces being tracked. That is, a large image with several tracked faces may
        /// take longer to process.
        /// </remarks>
        /// <param name="timer">Timer object invoking this call</param>
        private async void ProcessCurrentVideoFrame(ThreadPoolTimer timer)
        {
            if (this.currentState != ScenarioState.Streaming)
            {
                return;
            }

            // If a lock is being held it means we're still waiting for processing work on the previous frame to complete.
            // In this situation, don't wait on the semaphore but exit immediately.
            if (!frameProcessingSemaphore.Wait(0))
            {
                return;
            }

            try
            {
                IList<DetectedFace> faces = null;

                // Create a VideoFrame object specifying the pixel format we want our capture image to be (NV12 bitmap in this case).
                // GetPreviewFrame will convert the native webcam frame into this format.
                const BitmapPixelFormat InputPixelFormat = BitmapPixelFormat.Nv12;
                using (VideoFrame previewFrame = new VideoFrame(InputPixelFormat, (int)this.videoProperties.Width, (int)this.videoProperties.Height))
                {
                    await snapshotSemaphore.WaitAsync();
                    try
                    {
                        await this.mediaCapture.GetPreviewFrameAsync(previewFrame);

                        // Test10
                        System.Diagnostics.Debug.WriteLine("mediaCapture.GetRecordRotation : " + mediaCapture.GetRecordRotation());
                        System.Diagnostics.Debug.WriteLine("mediaCapture.GetPreviewRotation : " + mediaCapture.GetPreviewRotation());
                        //mediaCapture.

                    } finally
                    {
                        snapshotSemaphore.Release();    
                    }
                    // The returned VideoFrame should be in the supported NV12 format but we need to verify this.
                    if (FaceDetector.IsBitmapPixelFormatSupported(previewFrame.SoftwareBitmap.BitmapPixelFormat))
                    {
                        faces = await this.faceTracker.ProcessNextFrameAsync(previewFrame);
                    }
                    else
                    {
                        throw new System.NotSupportedException("PixelFormat '" + InputPixelFormat.ToString() + "' is not supported by FaceDetector");
                    }

                    // Create our visualization using the frame dimensions and face results but run it on the UI thread.
                    var previewFrameSize = new Windows.Foundation.Size(previewFrame.SoftwareBitmap.PixelWidth, previewFrame.SoftwareBitmap.PixelHeight);
                    var ignored = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                    {
                        this.SetupVisualization(previewFrameSize, faces);
                    });
                }
            }
            catch (Exception ex)
            {
                var ignored = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
                {
                    this.rootPage.NotifyUser(ex.ToString(), NotifyType.ErrorMessage);
                });
            }
            finally
            {
                frameProcessingSemaphore.Release();
            }
        }


// TODO Oana: remove maybe
     // detector
     // should run in background -> take pictures -> save them, one everytime person clicks on snapshot button...
     // -> perform face detection on them but don't show it
        /// <summary>
        /// Captures a single frame from the running webcam stream and executes the FaceDetector on the image. If successful calls SetupVisualization to display the results.
        /// </summary>
        /// <returns>Async Task object returning true if the capture was successful and false if an exception occurred.</returns>
        private async Task<bool> TakeSnapshotAndFindFaces()
        {
            bool successful = true;
   
            try
            {
                if (this.currentState != ScenarioState.Streaming)
                {
                    return false;
                }

                WriteableBitmap displaySource = null;
                IList<DetectedFace> faces = null;

                // Create a VideoFrame object specifying the pixel format we want our capture image to be (NV12 bitmap in this case).
                // GetPreviewFrame will convert the native webcam frame into this format.
                const BitmapPixelFormat InputPixelFormat = BitmapPixelFormat.Nv12;
                using (VideoFrame previewFrame = new VideoFrame(InputPixelFormat, (int)this.videoProperties.Width, (int)this.videoProperties.Height))
                {
                    await this.mediaCapture.GetPreviewFrameAsync(previewFrame);

                    // The returned VideoFrame should be in the supported NV12 format but we need to verify this.
                    if (FaceDetector.IsBitmapPixelFormatSupported(previewFrame.SoftwareBitmap.BitmapPixelFormat))
                    {
                        faces = await this.faceDetector.DetectFacesAsync(previewFrame.SoftwareBitmap);
                    }
                    else
                    {
                        this.rootPage.NotifyUser("PixelFormat '" + InputPixelFormat.ToString() + "' is not supported by FaceDetector", NotifyType.ErrorMessage);
                    }

                    // TODO: create png (?) save outside
                    // Create a WritableBitmap for our visualization display; copy the original bitmap pixels to wb's buffer.
                    // Note that WriteableBitmap doesn't support NV12 and we have to convert it to 32-bit BGRA.
                    using (SoftwareBitmap convertedSource = SoftwareBitmap.Convert(previewFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8))
                    {
                        displaySource = new WriteableBitmap(convertedSource.PixelWidth, convertedSource.PixelHeight);
                        convertedSource.CopyToBuffer(displaySource.PixelBuffer);
                    }
     // TODO Oana: change this, but insert it  - error because SetupVisualization method here takes different arguments... overload?
                    // Create our display using the available image and face results.
                    this.SetupVisualization2(displaySource, faces);
                }
            }
            catch (Exception ex)
            {
                this.rootPage.NotifyUser(ex.ToString(), NotifyType.ErrorMessage);
                successful = false;
            }

            return successful;
        }

     // TRACKER
        // Oana: Box zeichnen auf faces auf display
        /// <summary>
        /// Takes the webcam image and FaceTracker results and assembles the visualization onto the Canvas.
        /// </summary>
        /// <param name="framePizelSize">Width and height (in pixels) of the video capture frame</param>
        /// <param name="foundFaces">List of detected faces; output from FaceTracker</param>
        private void SetupVisualization(Windows.Foundation.Size framePizelSize, IList<DetectedFace> foundFaces)
        {
            // TeST
            faceIDsCurrFrame = new int[] { };
            faceIDsPrevFrame = new int[] { };

            this.VisualizationCanvas.Children.Clear();

            double actualWidth = this.VisualizationCanvas.ActualWidth;
            double actualHeight = this.VisualizationCanvas.ActualHeight;

            if (this.currentState == ScenarioState.Streaming && foundFaces != null && actualWidth != 0 && actualHeight != 0)
            {
                double widthScale = framePizelSize.Width / actualWidth;
                double heightScale = framePizelSize.Height / actualHeight;

                // Sort faces according to their X position
                foundFaces = foundFaces.OrderBy(face => face.FaceBox.X).ToList();

                // TeST
                if (firstFrame) {
    //                faceCounter = 0;
                    faceIDsCurrFrame = new int[foundFaces.Count];
                    faceIDsPrevFrame = new int[foundFaces.Count];
                    for (int i = 0; i < foundFaces.Count; i++) {
                        faceIDsCurrFrame[i] = i;
                        faceIDsPrevFrame[i] = i;
                    }
                }
    //            int count = faceCounter;
                faceIDsCurrFrame = new int[foundFaces.Count];
                faceIDsPrevFrame = new int[foundFaces.Count];

                int count = 0;

                foreach (DetectedFace face in foundFaces)
                {
                    // TeST
                    //
                    if (!firstFrame) {
                        faceIDsCurrFrame = getCorrectFaceIds(foundFaces, facesPrevFrame, faceIDsPrevFrame, faceMovementDirection);
                    }

                    // Create a rectangle element for displaying the face box but since we're using a Canvas
                    // we must scale the rectangles according to the frames's actual size.
                    Rectangle box = new Rectangle();
                    box.Width = (uint)(face.FaceBox.Width / widthScale);
                    box.Height = (uint)(face.FaceBox.Height / heightScale);
                    box.Fill = this.fillBrush;
                    box.Stroke = this.lineBrush;
                    box.StrokeThickness = this.lineThickness;
                    // positioning on canvas: textBlock.Margin = new Thickness(left, top, right, bottom);
                    box.Margin = new Thickness((uint)(face.FaceBox.X / widthScale), (uint)(face.FaceBox.Y / heightScale), 0, 0);

                    this.VisualizationCanvas.Children.Add(box);

                    // TEST
                  //  System.Diagnostics.Debug.WriteLine("x Position, FaceBox : " + face.FaceBox.X );

                    // Index of faces - Visualization: starting from 0, from left to right
                    TextBlock txtBlock = new TextBlock();
                    txtBlock.FontSize = 18;
                    txtBlock.Text = "" + faceIDsCurrFrame[count]/*count*/;  // TODO! - replace
                    txtBlock.Foreground = new SolidColorBrush(Windows.UI.Colors.Red);
                    txtBlock.Width = (uint)(face.FaceBox.Width / widthScale);
                    txtBlock.Height = (uint)(face.FaceBox.Height / heightScale);
                    // positioning on canvas: textBlock.Margin = new Thickness(left, top, right, bottom);
                    txtBlock.Margin = new Thickness((uint)(face.FaceBox.X / widthScale), (uint)(face.FaceBox.Y / heightScale), 0, 0);
                    this.VisualizationCanvas.Children.Add(txtBlock);

                    count++; // TODO! - replace

                    //TeST
                    facesPrevFrame = foundFaces;
                    faceIDsPrevFrame = faceIDsCurrFrame;

                    // TEST
                    System.Diagnostics.Debug.WriteLine("ids current frame: ");
                    for (int l = 0; l < faceIDsCurrFrame.Length; l++) {
                        System.Diagnostics.Debug.WriteLine(faceIDsCurrFrame[l]);
                    }

                    firstFrame = false;
                }
            }
        }

        // Self-written Algorithm, for unique ID allocation
        private int[] getCorrectFaceIds(IList<DetectedFace> foundFaces, IList<DetectedFace> facesPrevFrame, int[] faceIDsPrevFrame, Vector2 faceMoveDir) {
            int[] currectFaceID = new int[foundFaces.Count];

            // same number of faces
            if (foundFaces.Count == facesPrevFrame.Count) {
                for (int i = 0; i < foundFaces.Count; i++) {
                    // calculate new face movement direction
                    faceMovementDirection = new Vector2(foundFaces[i].FaceBox.X - facesPrevFrame[i].FaceBox.X, foundFaces[i].FaceBox.Y - facesPrevFrame[i].FaceBox.Y);
                }
                // indices remain the same as in the previous frame
                currectFaceID = faceIDsPrevFrame;
            }
            // more faces than before
            else if (foundFaces.Count > facesPrevFrame.Count) {
                // ( Annahme: Leute bleiben am selben Platz sitzen )

                // faces bewegen sich nach links
                // Camera bewegt sich nach rechts
                if (faceMovementDirection.X < 0){
                    // letzte vergebene ID holen
                    int k = faceIDsPrevFrame[faceIDsPrevFrame.Length - 1];
                    for (int j = (facesPrevFrame.Count - 1); j < foundFaces.Count; j++)
                    { // Itearion durch alle neu hinzugefuegten faces
                        currectFaceID[j] = ++k;
                    }
                }
                // faces bewegen sich nach rechts
                // Camera bewegt sich nach links
                else if (faceMovementDirection.X > 0) {
                    // die erste vergeben ID des letzten Frames holen
                    int k = faceIDsPrevFrame[0];
                    for (int j = 0; j < foundFaces.Count; j++) {
                        currectFaceID[j] = k++;
                    }
              /*      for (int j = (foundFaces.Count - facesPrevFrame.Count) - 1; j >= 0; j++)
                    { // Itearion durch alle neu hinzugefuegten faces
                        currectFaceID[j] = --k;
                    }
                    int l = 0;
                    for (int j = (foundFaces.Count - facesPrevFrame.Count); j < foundFaces.Count; j++)
                    {// Itearion durch alle restlichen faces, die gleich geblieben sind wie im letzten Frame
                        currectFaceID[j] = faceIDsPrevFrame[l++];
                    }*/
                }
                // keep the old face movement direction, until same number of faces in consecutive frames comes and it can be recalculated
                faceMovementDirection = faceMoveDir;
            }
            // less faces than before
            else if (foundFaces.Count < facesPrevFrame.Count) {
                // faces bewegen sich nach links
                // Camera bewegt sich nach rechts
                if (faceMovementDirection.X < 0){
                    // die erste in dieser Frame nicht weggeschnittene ID aus dem letzten Frame holen
                    int k = faceIDsPrevFrame[facesPrevFrame.Count - foundFaces.Count];
                    for (int j = 0; j < foundFaces.Count; j++) {
                        currectFaceID[j] =/* faceIDsPrevFrame[*/k++/*]*/;
                    }
                }
                // faces bewegen sich nach rechts
                // Camera bewegt sich nach links
                else if (faceMovementDirection.X > 0) {
                    // die erste vergeben ID des letzten Frames holen
                    int k = faceIDsPrevFrame[0];
                    for (int j = 0; j < foundFaces.Count; j++) {
                        currectFaceID[j] = /*faceIDsPrevFrame[*/k++/*]*/;
                    }
                }
                // keep the old face movement direction
                faceMovementDirection = faceMoveDir;
            }

            return currectFaceID;
        }

        //Oana: TODO: zeichne box auf face in PNG
        // DETECTOR
        /// <summary>
        /// Takes the webcam image and FaceDetector results and assembles the visualization onto the Canvas.
        /// </summary>
        /// <param name="displaySource">Bitmap object holding the image we're going to display</param>
        /// <param name="foundFaces">List of detected faces; output from FaceDetector</param>
        private void SetupVisualization2(WriteableBitmap displaySource, IList<DetectedFace> foundFaces)
        {

            // temp
            System.Diagnostics.Debug.WriteLine("Wow - SetupVisualization2");


            // TODO: adapt this method
            // TODO after: then use it to print images to folder
            
   /*         ImageBrush brush = new ImageBrush();
            brush.ImageSource = displaySource;
            brush.Stretch = Stretch.Fill;
            this.SnapshotCanvas.Background = brush;
            // try
        //    this.RenderTargetBitmap.Background = brush;

            if (foundFaces != null)
            {
                double widthScale = displaySource.PixelWidth / this.SnapshotCanvas.ActualWidth;
                double heightScale = displaySource.PixelHeight / this.SnapshotCanvas.ActualHeight;

                foreach (DetectedFace face in foundFaces)
                {
                    // Create a rectangle element for displaying the face box but since we're using a Canvas
                    // we must scale the rectangles according to the image's actual size.
                    // The original FaceBox values are saved in the Rectangle's Tag field so we can update the
                    // boxes when the Canvas is resized.
                    Rectangle box = new Rectangle();
                    box.Tag = face.FaceBox;
                    box.Width = (uint)(face.FaceBox.Width / widthScale);
                    box.Height = (uint)(face.FaceBox.Height / heightScale);
                    box.Fill = this.fillBrush;
                    box.Stroke = this.lineBrush;
                    box.StrokeThickness = this.lineThickness;
                    box.Margin = new Thickness((uint)(face.FaceBox.X / widthScale), (uint)(face.FaceBox.Y / heightScale), 0, 0);

                    this.SnapshotCanvas.Children.Add(box);
                }
            }

            string message;
            if (foundFaces == null || foundFaces.Count == 0)
            {
                message = "Didn't find any human faces in the image";
            }
            else if (foundFaces.Count == 1)
            {
                message = "Found a human face in the image";
            }
            else
            {
                message = "Found " + foundFaces.Count + " human faces in the image";
            }

            this.rootPage.NotifyUser(message, NotifyType.StatusMessage);
    */        
        }

        /// <summary>
        /// Manages the scenario's internal state. Invokes the internal methods and updates the UI according to the
        /// passed in state value. Handles failures and resets the state if necessary.
        /// </summary>
        /// <param name="newState">State to switch to</param>
        private async void ChangeScenarioState(ScenarioState newState)
        {
            // Disable UI while state change is in progress
            this.CameraStreamingButton.IsEnabled = false;

            switch (newState)
            {
                case ScenarioState.Idle:

                    this.ShutdownWebCam();

                    this.VisualizationCanvas.Children.Clear();
                    this.CameraSnapshotButton.IsEnabled = false;
                    this.CameraStreamingButton.Content = "Start Streaming";
                    this.CameraSnapshotButton.Content = "Take Snapshot";
                    this.currentState = newState;
                    break;

                case ScenarioState.Streaming:

                    if (!await this.StartWebcamStreaming())
                    {
                        this.ChangeScenarioState(ScenarioState.Idle);
                        break;
                    }

                    this.VisualizationCanvas.Children.Clear();
                    this.CameraSnapshotButton.IsEnabled = true;
                    this.CameraStreamingButton.Content = "Stop Streaming";
                    this.CameraSnapshotButton.Content = "Take Snapshot";
                    this.currentState = newState;
                    this.CameraStreamingButton.IsEnabled = true;
                    break;
            }
        }

        /// <summary>
        /// Handles MediaCapture stream failures by shutting down streaming and returning to Idle state.
        /// </summary>
        /// <param name="sender">The source of the event, i.e. our MediaCapture object</param>
        /// <param name="args">Event data</param>
        private void MediaCapture_CameraStreamFailed(MediaCapture sender, object args)
        {
            // MediaCapture is not Agile and so we cannot invoke its methods on this caller's thread
            // and instead need to schedule the state change on the UI thread.
            var ignored = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                ChangeScenarioState(ScenarioState.Idle);
            });
        }
        
        /// <summary>
        /// Handles MediaCapture changes by shutting down streaming and returning to Idle state.
        /// </summary>
        /// <param name="sender">The source of the event, i.e. our MediaCapture object</param>
        /// <param name="args">Event data</param>
        private void MediaCapture_CameraStreamStateChanged(MediaCapture sender, object args)
        {
            // MediaCapture is not Agile and so we cannot invoke it's methods on this caller's thread
            // and instead need to schedule the state change on the UI thread.
            var ignored = this.Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, () =>
            {
                ChangeScenarioState(ScenarioState.Idle);
            });
        }   

        /// <summary>
        /// Handles "streaming" button clicks to start/stop webcam streaming.
        /// </summary>
        /// <param name="sender">Button user clicked</param>
        /// <param name="e">Event data</param>
        private void CameraStreamingButton_Click(object sender, RoutedEventArgs e)
        {
            if (this.currentState == ScenarioState.Streaming)
            {
                this.rootPage.NotifyUser(string.Empty, NotifyType.StatusMessage);
                this.ChangeScenarioState(ScenarioState.Idle);
            }
            else
            {
                this.rootPage.NotifyUser(string.Empty, NotifyType.StatusMessage);
                this.ChangeScenarioState(ScenarioState.Streaming);
            }
        }
        
        /// <summary>
        /// Handles "snapshot" button clicks to take a snapshot or clear the current display.
        /// </summary>
        /// <param name="sender">Button user clicked</param>
        /// <param name="e">Event data</param>
        private async void CameraSnapshotButton_Click(object sender, RoutedEventArgs e)
        {
            // temp
            System.Diagnostics.Debug.WriteLine("WOW " + count++);
            //test
            bool myBool = await saveToImgFile();
            // test!
            System.Diagnostics.Debug.WriteLine(myBool);
        }

        /// <summary>
        /// Takes the webcam image and FaceDetector results and assembles the visualization into an image file format, saves it.
        /// </summary>
        /// <param name="displaySource">Bitmap object holding the image we're going to display</param>
        /// <param name="foundFaces">List of detected faces; output from FaceDetector</param>
        private async Task<bool>saveToImgFile()
        {
            // http://stackoverflow.com/questions/7612602/why-cant-i-use-the-await-operator-within-the-body-of-a-lock-statement

            bool successful = true;
            try
            {
                // sollte im Streaming phase sein... 
                if (this.currentState != ScenarioState.Streaming)
                {
                    return false;
                }
                WriteableBitmap displaySource = null;
                IList<DetectedFace> faces = null;

                // Create a VideoFrame object specifying the pixel format we want our capture image to be (NV12 bitmap in this case).
                // GetPreviewFrame will convert the native webcam frame into this format.
                const BitmapPixelFormat InputPixelFormat = BitmapPixelFormat.Nv12;
                using (VideoFrame previewFrame = new VideoFrame(InputPixelFormat, (int)this.videoProperties.Width, (int)this.videoProperties.Height))
                {
                    await snapshotSemaphore.WaitAsync();
                    try
                    {
                        // has to be used in async method - !
                        await this.mediaCapture.GetPreviewFrameAsync(previewFrame);
                    }
                    finally
                    {
                        snapshotSemaphore.Release();
                    }
                    
                    // The returned VideoFrame should be in the supported NV12 format but we need to verify this.
                    if (FaceDetector.IsBitmapPixelFormatSupported(previewFrame.SoftwareBitmap.BitmapPixelFormat))
                    {
                        faces = await this.faceDetector.DetectFacesAsync(previewFrame.SoftwareBitmap);
                    }
                    else
                    {
                        this.rootPage.NotifyUser("PixelFormat '" + InputPixelFormat.ToString() + "' is not supported by FaceDetector", NotifyType.ErrorMessage);
                    }
                    // Create png save outside
                    // Create a WritableBitmap for our visualization display; copy the original bitmap pixels to wb's buffer.
                    // Note that WriteableBitmap doesn't support NV12 and we have to convert it to 32-bit BGRA.
                    using (SoftwareBitmap convertedSource = SoftwareBitmap.Convert(previewFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8))
                    {
                        displaySource = new WriteableBitmap(convertedSource.PixelWidth, convertedSource.PixelHeight);
                        convertedSource.CopyToBuffer(displaySource.PixelBuffer);
                    }

                    // save as img format into Screenshots folder
                    StorageFile myImgFile = await writeableBitmapToFolder(displaySource);



                    // TODO 2: Visualization - save also with visualization?



                }
            }
            catch(Exception ex)
            {
                this.rootPage.NotifyUser(ex.ToString(), NotifyType.ErrorMessage);
                successful = false;
            }
            return successful;
        }

        // WriteableBitmap = unencodierte bilddaten
        // Compression
        // Encoder
        private async Task<StorageFile> writeableBitmapToFolder(WriteableBitmap WB)
        {
            // http://stackoverflow.com/questions/17140774/how-to-save-a-writeablebitmap-as-a-file

            string FileName = "MyFile"+count+".";
            Guid BitmapEncoderGuid = BitmapEncoder.PngEncoderId;
            FileName += "png"; //fileFormat;

            // Create directory to save screenshots in
            string myLocalPath = ApplicationData.Current.LocalFolder.Path;
            string myScreenshotPath = System.IO.Path.Combine(myLocalPath, "Screenshots");
            StorageFolder myFolder = ApplicationData.Current.LocalFolder;
            // Determine whether the directory exists
            if (!(Directory.Exists(myScreenshotPath)))
            {
                myFolder = await ApplicationData.Current.LocalFolder.CreateFolderAsync("Screenshots");
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("That path exists already.");
                myFolder = await StorageFolder.GetFolderFromPathAsync(myScreenshotPath);
            }
            // Compress file and write it to folder
            var file = await myFolder.CreateFileAsync(FileName, CreationCollisionOption.GenerateUniqueName);
            using (IRandomAccessStream stream = await file.OpenAsync(FileAccessMode.ReadWrite))
            {
                BitmapEncoder encoder = await BitmapEncoder.CreateAsync(BitmapEncoderGuid, stream);
                Stream pixelStream = WB.PixelBuffer.AsStream();
                byte[] pixels = new byte[pixelStream.Length];
                await pixelStream.ReadAsync(pixels, 0, pixels.Length);

                encoder.SetPixelData(BitmapPixelFormat.Bgra8, BitmapAlphaMode.Ignore,
                                    (uint)WB.PixelWidth,
                                    (uint)WB.PixelHeight,
                                    96.0,
                                    96.0,
                                    pixels);
                await encoder.FlushAsync();
            }
            return file; // necessary?
        } 
    }

 /*   class CameraRotationHelper
    {
        private EnclosureLocation _cameraEnclosureLocation;
        private DisplayInformation _displayInformation = DisplayInformation.GetForCurrentView();
        private SimpleOrientationSensor _orientationSensor = SimpleOrientationSensor.GetDefault();
        public event EventHandler<bool> OrientationChanged;

        public CameraRotationHelper(EnclosureLocation cameraEnclosureLocation)
        {
            _cameraEnclosureLocation = cameraEnclosureLocation;
            if (!IsEnclosureLocationExternal(_cameraEnclosureLocation))
            {
                _orientationSensor.OrientationChanged += SimpleOrientationSensor_OrientationChanged;
            }
            _displayInformation.OrientationChanged += DisplayInformation_OrientationChanged;
        }

        private void SimpleOrientationSensor_OrientationChanged(SimpleOrientationSensor sender, SimpleOrientationSensorOrientationChangedEventArgs args)
        {
            if (args.Orientation != SimpleOrientation.Faceup && args.Orientation != SimpleOrientation.Facedown)
            {
                HandleOrientationChanged(false);
            }
        }

        private void DisplayInformation_OrientationChanged(DisplayInformation sender, object args)
        {
            HandleOrientationChanged(true);
        }

        private void HandleOrientationChanged(bool updatePreviewStreamRequired)
        {
            var handler = OrientationChanged;
            if (handler != null)
            {
                handler(this, updatePreviewStreamRequired);
            }
        }

        public static bool IsEnclosureLocationExternal(EnclosureLocation enclosureLocation)
        {
            return (enclosureLocation == null || enclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Unknown);
        }

        private bool IsCameraMirrored()
        {
            // Front panel cameras are mirrored by default
            return (_cameraEnclosureLocation.Panel == Windows.Devices.Enumeration.Panel.Front);
        }

        private SimpleOrientation GetCameraOrientationRelativeToNativeOrientation()
        {
            // Get the rotation angle of the camera enclosure
            var enclosureAngle = ConvertClockwiseDegreesToSimpleOrientation((int)_cameraEnclosureLocation.RotationAngleInDegreesClockwise);

            // Account for the fact that, on portrait-first devices, the built in camera sensor is read at a 90 degree offset to the native orientation
            if (_displayInformation.NativeOrientation == DisplayOrientations.Portrait && !IsEnclosureLocationExternal(_cameraEnclosureLocation))
            {
                return AddOrientations(SimpleOrientation.Rotated90DegreesCounterclockwise, enclosureAngle);
            }
            else
            {
                return AddOrientations(SimpleOrientation.NotRotated, enclosureAngle);
            }
        }

        // Gets the rotation to rotate ui elements
        public SimpleOrientation GetUIOrientation()
        {
            if (IsEnclosureLocationExternal(_cameraEnclosureLocation))
            {
                // Cameras that are not attached to the device do not rotate along with it, so apply no rotation
                return SimpleOrientation.NotRotated;
            }

            // Return the difference between the orientation of the device and the orientation of the app display
            var deviceOrientation = _orientationSensor.GetCurrentOrientation();
            var displayOrientation = ConvertDisplayOrientationToSimpleOrientation(_displayInformation.CurrentOrientation);
            return SubOrientations(displayOrientation, deviceOrientation);
        }

        // Gets the rotation of the camera to rotate pictures/videos when saving to file
        public SimpleOrientation GetCameraCaptureOrientation()
        {
            if (IsEnclosureLocationExternal(_cameraEnclosureLocation))
            {
                // Cameras that are not attached to the device do not rotate along with it, so apply no rotation
                return SimpleOrientation.NotRotated;
            }

            // Get the device orienation offset by the camera hardware offset
            var deviceOrientation = _orientationSensor.GetCurrentOrientation();
            var result = SubOrientations(deviceOrientation, GetCameraOrientationRelativeToNativeOrientation());

            // If the preview is being mirrored for a front-facing camera, then the rotation should be inverted
            if (IsCameraMirrored())
            {
                result = MirrorOrientation(result);
            }
            return result;
        }

        // Gets the rotation of the camera to display the camera preview
        public SimpleOrientation GetCameraPreviewOrientation()
        {
            if (IsEnclosureLocationExternal(_cameraEnclosureLocation))
            {
                // Cameras that are not attached to the device do not rotate along with it, so apply no rotation
                return SimpleOrientation.NotRotated;
            }

            // Get the app display rotation offset by the camera hardware offset
            var result = ConvertDisplayOrientationToSimpleOrientation(_displayInformation.CurrentOrientation);
            result = SubOrientations(result, GetCameraOrientationRelativeToNativeOrientation());

            // If the preview is being mirrored for a front-facing camera, then the rotation should be inverted
            if (IsCameraMirrored())
            {
                result = MirrorOrientation(result);
            }
            return result;
        }

        public static PhotoOrientation ConvertSimpleOrientationToPhotoOrientation(SimpleOrientation orientation)
        {
            switch (orientation)
            {
                case SimpleOrientation.Rotated90DegreesCounterclockwise:
                    return PhotoOrientation.Rotate90;
                case SimpleOrientation.Rotated180DegreesCounterclockwise:
                    return PhotoOrientation.Rotate180;
                case SimpleOrientation.Rotated270DegreesCounterclockwise:
                    return PhotoOrientation.Rotate270;
                case SimpleOrientation.NotRotated:
                default:
                    return PhotoOrientation.Normal;
            }
        }

        public static int ConvertSimpleOrientationToClockwiseDegrees(SimpleOrientation orientation)
        {
            switch (orientation)
            {
                case SimpleOrientation.Rotated90DegreesCounterclockwise:
                    return 270;
                case SimpleOrientation.Rotated180DegreesCounterclockwise:
                    return 180;
                case SimpleOrientation.Rotated270DegreesCounterclockwise:
                    return 90;
                case SimpleOrientation.NotRotated:
                default:
                    return 0;
            }
        }

        private SimpleOrientation ConvertDisplayOrientationToSimpleOrientation(DisplayOrientations orientation)
        {
            SimpleOrientation result;
            switch (orientation)
            {
                case DisplayOrientations.Landscape:
                    result = SimpleOrientation.NotRotated;
                    break;
                case DisplayOrientations.PortraitFlipped:
                    result = SimpleOrientation.Rotated90DegreesCounterclockwise;
                    break;
                case DisplayOrientations.LandscapeFlipped:
                    result = SimpleOrientation.Rotated180DegreesCounterclockwise;
                    break;
                case DisplayOrientations.Portrait:
                default:
                    result = SimpleOrientation.Rotated270DegreesCounterclockwise;
                    break;
            }

            // Above assumes landscape; offset is needed if native orientation is portrait
            if (_displayInformation.NativeOrientation == DisplayOrientations.Portrait)
            {
                result = AddOrientations(result, SimpleOrientation.Rotated90DegreesCounterclockwise);
            }

            return result;
        }

        private static SimpleOrientation MirrorOrientation(SimpleOrientation orientation)
        {
            // This only affects the 90 and 270 degree cases, because rotating 0 and 180 degrees is the same clockwise and counter-clockwise
            switch (orientation)
            {
                case SimpleOrientation.Rotated90DegreesCounterclockwise:
                    return SimpleOrientation.Rotated270DegreesCounterclockwise;
                case SimpleOrientation.Rotated270DegreesCounterclockwise:
                    return SimpleOrientation.Rotated90DegreesCounterclockwise;
            }
            return orientation;
        }

        private static SimpleOrientation AddOrientations(SimpleOrientation a, SimpleOrientation b)
        {
            var aRot = ConvertSimpleOrientationToClockwiseDegrees(a);
            var bRot = ConvertSimpleOrientationToClockwiseDegrees(b);
            var result = (aRot + bRot) % 360;
            return ConvertClockwiseDegreesToSimpleOrientation(result);
        }

        private static SimpleOrientation SubOrientations(SimpleOrientation a, SimpleOrientation b)
        {
            var aRot = ConvertSimpleOrientationToClockwiseDegrees(a);
            var bRot = ConvertSimpleOrientationToClockwiseDegrees(b);
            //add 360 to ensure the modulus operator does not operate on a negative
            var result = (360 + (aRot - bRot)) % 360;
            return ConvertClockwiseDegreesToSimpleOrientation(result);
        }

        private static VideoRotation ConvertSimpleOrientationToVideoRotation(SimpleOrientation orientation)
        {
            switch (orientation)
            {
                case SimpleOrientation.Rotated90DegreesCounterclockwise:
                    return VideoRotation.Clockwise270Degrees;
                case SimpleOrientation.Rotated180DegreesCounterclockwise:
                    return VideoRotation.Clockwise180Degrees;
                case SimpleOrientation.Rotated270DegreesCounterclockwise:
                    return VideoRotation.Clockwise90Degrees;
                case SimpleOrientation.NotRotated:
                default:
                    return VideoRotation.None;
            }
        }

        private static SimpleOrientation ConvertClockwiseDegreesToSimpleOrientation(int orientation)
        {
            switch (orientation)
            {
                case 270:
                    return SimpleOrientation.Rotated90DegreesCounterclockwise;
                case 180:
                    return SimpleOrientation.Rotated180DegreesCounterclockwise;
                case 90:
                    return SimpleOrientation.Rotated270DegreesCounterclockwise;
                case 0:
                default:
                    return SimpleOrientation.NotRotated;
            }
        }
    }
    */
}
