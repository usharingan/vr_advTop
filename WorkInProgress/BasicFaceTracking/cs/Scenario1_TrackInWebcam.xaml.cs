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
        int count = 0;

        private SemaphoreSlim snapshotSemaphore = new SemaphoreSlim(1);

        /// <summary>
        /// Constructor
        /// Initializes a new instance of the <see cref="TrackFacesInWebcam"/> class.
        /// </summary>
        public TrackFacesInWebcam()
        {
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
            Streaming //,

            /// <summary>
            /// Snapshot image has been captured and is being displayed along with detected faces; webcam is not active. 
        /// -> should not be displayed tho -> do smth else -> save img as PNG?
            /// </summary>
      //      Snapshot  //...
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

      // Oana: see if needed... and for what
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
                this.mediaCapture = new MediaCapture();

                // For this scenario, we only need Video (not microphone) so specify this in the initializer.
                // NOTE: the appxmanifest only declares "webcam" under capabilities and if this is changed to include
                // microphone (default constructor) you must add "microphone" to the manifest or initialization will fail.
                MediaCaptureInitializationSettings settings = new MediaCaptureInitializationSettings();
                settings.StreamingCaptureMode = StreamingCaptureMode.Video;
                await this.mediaCapture.InitializeAsync(settings);
                this.mediaCapture.Failed += this.MediaCapture_CameraStreamFailed;
            //  this.mediaCapture.CameraStreamStateChanged += this.MediaCapture_CameraStreamStateChanged;

                // Cache the media properties as we'll need them later.
                var deviceController = this.mediaCapture.VideoDeviceController;
                this.videoProperties = deviceController.GetMediaStreamProperties(MediaStreamType.VideoPreview) as VideoEncodingProperties;

                // Immediately start streaming to our CaptureElement UI.
                // NOTE: CaptureElement's Source must be set before streaming is started.
                this.CamPreview.Source = this.mediaCapture;
                await this.mediaCapture.StartPreviewAsync();

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

  // Oana: should run in front
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


     // detector
     // Oana: should run in background -> take pictures -> save them, one everytime person clicks on snapshot button...
     // -> perform face detection on them but don't show it
     // TODO
        /// <summary>
        /// Captures a single frame from the running webcam stream and executes the FaceDetector on the image. If successful calls SetupVisualization to display the results.
        /// </summary>
        /// <returns>Async Task object returning true if the capture was successful and false if an exception occurred.</returns>
        private async Task<bool> TakeSnapshotAndFindFaces()
        {
            bool successful = true;
   /*
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
*/
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
            this.VisualizationCanvas.Children.Clear();

            double actualWidth = this.VisualizationCanvas.ActualWidth;
            double actualHeight = this.VisualizationCanvas.ActualHeight;

            if (this.currentState == ScenarioState.Streaming && foundFaces != null && actualWidth != 0 && actualHeight != 0)
            {
                double widthScale = framePizelSize.Width / actualWidth;
                double heightScale = framePizelSize.Height / actualHeight;

                foreach (DetectedFace face in foundFaces)
                {
                    // Create a rectangle element for displaying the face box but since we're using a Canvas
                    // we must scale the rectangles according to the frames's actual size.
                    Rectangle box = new Rectangle();
                    box.Width = (uint)(face.FaceBox.Width / widthScale);
                    box.Height = (uint)(face.FaceBox.Height / heightScale);
                    box.Fill = this.fillBrush;
                    box.Stroke = this.lineBrush;
                    box.StrokeThickness = this.lineThickness;
                    box.Margin = new Thickness((uint)(face.FaceBox.X / widthScale), (uint)(face.FaceBox.Y / heightScale), 0, 0);

                    this.VisualizationCanvas.Children.Add(box);
                }
            }
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



            /*
            ImageBrush brush = new ImageBrush();
            brush.ImageSource = displaySource;
            brush.Stretch = Stretch.Fill;
            this.SnapshotCanvas.Background = brush;

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

          /*      case ScenarioState.Snapshot:
                    // ?
                    // TODO
                    // temp
                    System.Diagnostics.Debug.WriteLine("Check");
                    //   System.Console.WriteLine("Wow");

                    // TODO: Sondercase
                    if (!await this.TakeSnapshotAndFindFaces())
                    {
                        this.ChangeScenarioState(ScenarioState.Idle);
                        break;
                    }
                    
                    // ok
                    this.VisualizationCanvas.Children.Clear();
                    this.CameraStreamingButton.Content = "Stop Streaming";
                    this.CameraSnapshotButton.IsEnabled = true;
                    this.CameraSnapshotButton.Content = "Take Snapshot";
                    this.currentState = newState;
                    this.CameraStreamingButton.IsEnabled = true;
                    break;
            */
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
            // System.Diagnostics.Debug.WriteLine("WOW " + count++);
            //test
            bool myBool = await saveToImgFile();
            // test!
            System.Diagnostics.Debug.WriteLine(myBool);

            // TODO HERE: 
            /*
                - extract screenshot picture
                - save picture as it is in image format
                - find faces in it
                - draw faces in
                - save picture withdrawn in faces in image format

             */


            /*        if (this.currentState == ScenarioState.Streaming)
                    {
                        this.rootPage.NotifyUser(string.Empty, NotifyType.StatusMessage);
                        this.ChangeScenarioState(ScenarioState.Snapshot);
                    }
                    else
                    {
                        this.rootPage.NotifyUser(string.Empty, NotifyType.StatusMessage);
                        this.ChangeScenarioState(ScenarioState.Idle);
                    }  
             */
        }



        /* in case of necessity...
           /// Updates any existing face bounding boxes in response to changes in the size of the Canvas.
        /// <summary>
        /// Updates any existing face bounding boxes in response to changes in the size of the Canvas.
        /// </summary>
        /// <param name="sender">Canvas whose size has changed</param>
        /// <param name="e">Event data</param>
        private void SnapshotCanvas_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            try
            {
                // If the Canvas is resized we must recompute a new scaling factor and
                // apply it to each face box.
                if (this.currentState == ScenarioState.Snapshot && this.SnapshotCanvas.Background != null)
                {
                    WriteableBitmap displaySource = (this.SnapshotCanvas.Background as ImageBrush).ImageSource as WriteableBitmap;

                    double widthScale = displaySource.PixelWidth / this.SnapshotCanvas.ActualWidth;
                    double heightScale = displaySource.PixelHeight / this.SnapshotCanvas.ActualHeight;

                    foreach (var item in this.SnapshotCanvas.Children)
                    {
                        Rectangle box = item as Rectangle;
                        if (box == null)
                        {
                            continue;
                        }

                        // We saved the original size of the face box in the rectangles Tag field.
                        BitmapBounds faceBounds = (BitmapBounds)box.Tag;
                        box.Width = (uint)(faceBounds.Width / widthScale);
                        box.Height = (uint)(faceBounds.Height / heightScale);

                        box.Margin = new Thickness((uint)(faceBounds.X / widthScale), (uint)(faceBounds.Y / heightScale), 0, 0);
                    }
                }
            }
            catch (Exception ex)
            {
                this.rootPage.NotifyUser(ex.ToString(), NotifyType.ErrorMessage);
            }
        }
         */

        /// <summary>
        /// Takes the webcam image and FaceDetector results and assembles the visualization into an image file format, saves it.
        /// </summary>
        /// <param name="displaySource">Bitmap object holding the image we're going to display</param>
        /// <param name="foundFaces">List of detected faces; output from FaceDetector</param>
        private async Task<bool>saveToImgFile(/*WriteableBitmap displaySource, IList<DetectedFace> foundFaces*/)
        {
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
                        // has to be used in async method
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
                    // TODO: create png (?) save outside
                    // Create a WritableBitmap for our visualization display; copy the original bitmap pixels to wb's buffer.
                    // Note that WriteableBitmap doesn't support NV12 and we have to convert it to 32-bit BGRA.
                    using (SoftwareBitmap convertedSource = SoftwareBitmap.Convert(previewFrame.SoftwareBitmap, BitmapPixelFormat.Bgra8))
                    {
                        displaySource = new WriteableBitmap(convertedSource.PixelWidth, convertedSource.PixelHeight);
                        convertedSource.CopyToBuffer(displaySource.PixelBuffer);
                    }

                    // TODO 1: try save as img format
                    StorageFile myImgFile = await writeableBitmapToStorageFile(displaySource);

                    // TODO 1/2: test save
                    saveImgStorageFileToFolder(myImgFile);

                    // temp
              //      System.Diagnostics.Debug.WriteLine(Directory.GetCurrentDirectory());

                    // does not work... -> save to array of StorageFiles?...


                    // TODO 2: Visualization

                 /*   //try
                    BitmapImage img = new BitmapImage();
                    img = await LoadImage(myImgFile);
//                    myImage.Source = img;
*/

                }
            }
            catch(Exception ex)
            {
                this.rootPage.NotifyUser(ex.ToString(), NotifyType.ErrorMessage);
                successful = false;
            }
            return successful;
        }

     /*   private static async Task<BitmapImage> LoadImage(StorageFile file)
        {
            BitmapImage bitmapImage = new BitmapImage();
            FileRandomAccessStream stream = (FileRandomAccessStream)await file.OpenAsync(FileAccessMode.Read);

            bitmapImage.SetSource(stream);

            return bitmapImage;
        } */

        //
        private async Task<StorageFile> writeableBitmapToStorageFile(WriteableBitmap WB/*, string fileFormat*/)
        {
            // http://stackoverflow.com/questions/17140774/how-to-save-a-writeablebitmap-as-a-file

            string FileName = "MyFile"+count+".";
            Guid BitmapEncoderGuid = BitmapEncoder.PngEncoderId;
            FileName += "png"; //fileFormat;

            var file = await Windows.Storage.ApplicationData.Current.TemporaryFolder.CreateFileAsync(FileName, CreationCollisionOption.GenerateUniqueName);
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
            return file;
        }


        private async void saveImgStorageFileToFolder(StorageFile file)
        {
            // http://stackoverflow.com/questions/36550122/how-do-i-create-a-folder-in-a-uwp-application

            // debug - see where this folder is located, which we also have read/write permissions for
            string pathStringParent = ApplicationData.Current.LocalFolder.Path;
            System.Diagnostics.Debug.WriteLine(pathStringParent);

            // path to new created Screenshots folder
            string pathStringChild = System.IO.Path.Combine(pathStringParent, "Screenshots");

            // Determine whether the directory exists
            if (!(Directory.Exists(pathStringChild)))
            {
                StorageFolder myFolder = await ApplicationData.Current.LocalFolder.CreateFolderAsync("Screenshots");
                // TEST, Debug
                myFolder = ApplicationData.Current.LocalFolder;
                // debug
                System.Diagnostics.Debug.WriteLine("The directory was created successfully at {0}.", Directory.GetCreationTime(pathStringChild));
            }
            else {
                System.Diagnostics.Debug.WriteLine("That path exists already.");
            }

            try {
                // try to save storage file to 
                var stream = await file.OpenStreamForReadAsync();

                // StorageFolder myFolder = await ApplicationData.Current.LocalFolder.GetFolderAsync("Screenshots");
                // myFile = file = storageFile
                using (Stream outputStream = await file.OpenStreamForWriteAsync())
                {
         // TODO: solve first error first, then fix this
         //           await stream.CopyToAsync(outputStream);
                }
            }
            catch (IOException e) {
                System.Diagnostics.Debug.WriteLine(e.Message);
                System.Diagnostics.Debug.WriteLine(e.StackTrace);
            } catch(Exception e)
            {
                System.Diagnostics.Debug.WriteLine(e.Message);
                System.Diagnostics.Debug.WriteLine(e.StackTrace);
            }

        }

   /*     public async Task SaveToLocalFolderAsync(Stream stream, string fileName) 
        {
            StorageFolder localFolder = ApplicationData.Current.LocalFolder;
            StorageFile storageFile = await localFolder.CreateFileAsync(fileName, CreationCollisionOption.ReplaceExisting);
            using (Stream outputStream = await storageFile.OpenStreamForWriteAsync())
            {
                await stream.CopyToAsync(outputStream);
            }
        }
        */




        // detector
        // Oana: should run in background -> take pictures -> save them, one everytime person clicks on snapshot button...
        // -> perform face detection on them but don't show it
        // TODO

        /// <summary>
        /// Captures a single frame from the running webcam stream and executes the FaceDetector on the image. If successful calls SetupVisualization to display the results.
        /// </summary>
        /// <returns>Async Task object returning true if the capture was successful and false if an exception occurred.</returns>
        /*     private async Task<bool> TakeSnapshotAndFindFaces()
             {
                 bool successful = true;
                 /*
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
              *
                 return successful;
             }
             */
    }
}
