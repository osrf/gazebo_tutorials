# Using GzWeb's SDF viewer

The SDF viewer is a part of Gzweb that visualize robots without the need to a backend like gzserver (gazebo).

In this example, we will describe how to embed gzweb's SDF viewer into other web pages.

## Code
create file.

`gedit sdf_viewer_example.html`

and add the following content ([sdf_viewer_example.html](https://bitbucket.org/osrf/gzweb/raw/57afdee589e7d757ea7fd0cb011a5596a01aeed4/examples/view_sdf_from_url.html))

```html
<!DOCTYPE html>
<html lang="en">

  <head>
    <title>View SDF from URL</title>
    <meta charset="utf-8">
    <script src="../gz3d/build/gz3d.js"></script>
    <style>
          body {
        padding: 2em;
        font-family: helvetica;
      }
      #container {
        padding: 2em;
        height: 30em;
        width: 80%;
      }
      button {
        width: 8%;
      }
      input {
        width: 85%;
      }
    </style>
  </head>

  <body>

    <h2>
      View SDF from URL
    </h2>

    <p>
      This example shows how to visualize an SDF file on the browser by passing the URLs of its resources.
    </p>
    <p>
      The SDF parsing happens completely on the browser, so there is no need for a GzWeb server to be runnning in the backend.
    </p>

    <p>
      You can test it as follows:
    </p>

    <ol>
      <li>Under "Resources URLs", type the URL to a resource needed by the model
          (like meshes, textures, etc). If no resources are missing, skip to step 4.</li>
      <li>Click "Add URL"</li>
      <li>Go back to step 1</li>
      <li>Under "SDF model URL", fill the full URL to the model's SDF file</li>
      <li>Click "Load model!"</li>
    </ol>

    <div>
      <div>
        <h4>Resources URLs</h4>
        <input type="text" id="resources-input"></input>
        <button onclick="addURL()" id="add-button">Add URL</button>
      </div>
      <div id="resource-list">
      </div>
      <div>
        <h4>SDF model URL</h4>
        <input type="text" id="sdf-input"></input>
        <button onclick="loadModel()" id="load-button">Load model!</button>
      </div>
      <div id="sdf-display">
      </div>
    </div>

    <center>
      <div id="container">
      </div>
    </center>

    <script>
      if (!Detector.webgl)
        Detector.addGetWebGLMessage();

      var scene;
      var sdfparser;
      var resourcesInput;
      var sdfInput;
      var resourceList;

      init();
      animate();

      // Initialization
      function init()
      {
        // Initialize objects
        var shaders = new GZ3D.Shaders();
        scene = new GZ3D.Scene(shaders);
        sdfparser = new GZ3D.SdfParser(scene);
        sdfparser.usingFilesUrls = true;

        // Append to dom
        var container = document.getElementById('container');
        container.appendChild(scene.getDomElement());

        // Handle window resize
        window.addEventListener('resize', onWindowResize, false);
        onWindowResize();

        // Get DOM elements
        resourcesInput = document.getElementById('resources-input');
        sdfInput = document.getElementById('sdf-input');
        resourceList = document.getElementById('resource-list');

        animate();
      }

      // Callback when window is resized
      function onWindowResize()
      {
        scene.setSize(container.clientWidth, container.clientHeight);
      }

      // Recursively called animation loop
      function animate()
      {
        requestAnimationFrame(animate);
        scene.render();
      }

      // Callback when Load Model is pressed
      function loadModel()
      {
        var obj = sdfparser.loadSDF(sdfInput.value);
        if (!obj)
        {
          alert('Failed to load model, check the URL and try again');
          return;
        }
        scene.add(obj);

        // Hide inputs
        resourcesInput.style.display = 'none';
        sdfInput.style.display = 'none';
        document.getElementById('add-button').style.display = 'none';
        document.getElementById('load-button').style.display = 'none';

        sdfDisplay = document.getElementById('sdf-display');
        sdfDisplay.innerHTML = sdfDisplay.innerHTML + "<br>" + sdfInput.value;
      }

      // Callback when Add URL is pressed
      function addURL()
      {
        sdfparser.addUrl(resourcesInput.value);

        resourceList.innerHTML = resourceList.innerHTML + "<br>" + resourcesInput.value;
      }
    </script>

  </body>
</html>
```
## Code explained
First, we load `gz3d.js` which is the only script we will need to include to get the `SDF viewer` working.
``` html
<script src="../gz3d/build/gz3d.js"></script>
```
Assuming you have successfully installed Gzweb, you can find the installation instructions [here](http://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb), you will find `gz3d.js` in `<gzweb directory>/gz3d/build/gz3d.js`



read the nessesary `urls` to load the model.

``` html
<div>
  <h4>Resources URLs</h4>
  <input type="text" id="resources-input"></input>
  <button onclick="addURL()" id="add-button">Add URL</button>
</div>
```
```html
<div>
  <h4>SDF model URL</h4>
  <input type="text" id="sdf-input"></input>
  <button onclick="loadModel()" id="load-button">Load model!</button>
</div>
```

we create a `dom` element which we will append the `renderer` to.
```html
<div id="container">
</div>
```

### initialize the SDF viewer

 - it is required to instantiate `shaders`, `scene` and `sdfParser` objects.

 - set `usingFilesUrls` in the `sdfparser` to `true`, the API that supports loading the files themselves is still under development.

 - append the `renderer` to a dom element to appear.

 - call `animate` to start drawing the `scene`.

```js
var scene;
var sdfparser;
var resourcesInput;
var sdfInput;
var resourceList;

init();
animate();

// Initialization
function init()
{
  // Initialize objects
  var shaders = new GZ3D.Shaders();
  scene = new GZ3D.Scene(shaders);
  sdfparser = new GZ3D.SdfParser(scene);
  sdfparser.usingFilesUrls = true;

  // Append to dom
  var container = document.getElementById('container');
  container.appendChild(scene.getDomElement());

  // Handle window resize
  window.addEventListener('resize', onWindowResize, false);
  onWindowResize();

  // Get DOM elements
  resourcesInput = document.getElementById('resources-input');
  sdfInput = document.getElementById('sdf-input');
  resourceList = document.getElementById('resource-list');

  animate();
}

// Recursively called animation loop
function animate()
{
  requestAnimationFrame(animate);
  scene.render();
}
```
the `sdfparser.loadSDF()` takes the model `sdf` file as an argument and returns the model as an `object`.

we then simply call `scene.add(<model object>)` to add the `model` to the `scene`.
```js
// Callback when Load Model is pressed
function loadModel()
{
  var obj = sdfparser.loadSDF(sdfInput.value);
  if (!obj)
  {
    alert('Failed to load model, check the URL and try again');
    return;
  }
  scene.add(obj);

  // Hide inputs
  resourcesInput.style.display = 'none';
  sdfInput.style.display = 'none';
  document.getElementById('add-button').style.display = 'none';
  document.getElementById('load-button').style.display = 'none';

  sdfDisplay = document.getElementById('sdf-display');
  sdfDisplay.innerHTML = sdfDisplay.innerHTML + "<br>" + sdfInput.value;
}
```

in `addUrl` we simply call `sdfparser.addUrl(<resources urls>)` (e.g. textures, meshes), this shouldn't be called after `sdfparser.loadSDF(<model sdf file url>)`.

```js
// Callback when Add URL is pressed
function addURL()
{
  sdfparser.addUrl(resourcesInput.value);

  resourceList.innerHTML = resourceList.innerHTML + "<br>" + resourcesInput.value;
}
```