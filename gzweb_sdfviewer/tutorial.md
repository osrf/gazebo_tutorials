# Using GzWeb's SDF viewer

The SDF viewer is a part of Gzweb that visualize robots without the need to a backend like gzserver (gazebo).

In this example, we will describe how to embed gzweb's SDF viewer into other web pages.

## Code
Create a new `html` file.

`gedit sdf_viewer_example.html`

Add the following content ([sdf_viewer_example.html](https://bitbucket.org/osrf/gzweb/raw/57afdee589e7d757ea7fd0cb011a5596a01aeed4/examples/view_sdf_from_url.html))

<include src='https://bitbucket.org/osrf/gzweb/raw/gzweb_1.4.0/examples/view_sdf_from_url.html' />

## Code explained

First, we load `gz3d.js` which is the only script we will need to include to get the `SDF viewer` working.

```html
<script src="../gz3d/build/gz3d.js"></script>
```

Assuming you have successfully installed Gzweb, you can find the installation instructions [here](http://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb), you will find `gz3d.js` in `<gzweb directory>/gz3d/build/gz3d.js`


Read the nessesary `urls` to load the model.

```html
<div>
  <h4>Resources URLs</h4>
  <input type="text" id="resources-input"></input>
  <button onclick="addURL()" id="add-button">Add URL</button>
</div>
```

We create a `dom` element which we will append the `renderer` to.

```html
<div id="container">
</div>
```

### Initialize the SDF viewer

 - It is required to instantiate `shaders`, `scene` and `sdfParser` objects.

 - Set `usingFilesUrls` in the `sdfparser` to `true`, the API that supports loading the files themselves is still under development.

 - Append the `renderer` to a dom element to appear.

 - Call `animate` to start drawing the `scene`.

<include from="/var scene;/" to="/scene.render\(\);\*\}\/" src='https://bitbucket.org/osrf/gzweb/raw/gzweb_1.4.0/examples/view_sdf_from_url.html' />

The `sdfparser.loadSDF()` takes the model `sdf` file as an argument and returns the model as an `object`.

We then simply call `scene.add(<model object>)` to add the `model` to the `scene`, the object is a (`THREE.Object3D`)[https://threejs.org/docs/#api/core/Object3D].

<include from="/\/\/ Callback/" to="/sdfInput.value;\*\}\/" src='https://bitbucket.org/osrf/gzweb/raw/gzweb_1.4.0/examples/view_sdf_from_url.html' />

In `addUrl()` function we call `sdfparser.addUrl(<resources urls>)` (e.g. textures, meshes), this shouldn't be called after `sdfparser.loadSDF(<model sdf file url>)`.

```js
// Callback when Add URL is pressed
function addURL()
{
  sdfparser.addUrl(resourcesInput.value);

  resourceList.innerHTML = resourceList.innerHTML + "<br>" + resourcesInput.value;
}
```