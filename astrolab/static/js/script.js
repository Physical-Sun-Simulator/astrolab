const apiURLBase = "https://nominatim.openstreetmap.org/search?q=";
const menuWidth = "400px"
const baseLatitude = 51.4486602;
const baseLongitude = 5.4903995655080475;
const baseZoomLevel = 17;
var openMenu = false;
var map = L.map('map').setView([baseLatitude, baseLongitude], baseZoomLevel);
L.marker([baseLatitude, baseLongitude]).addTo(map);
setupMap();

function setupMap() {
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 20,
        attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    }).addTo(map);
}

function changeMenu() {
    if (!openMenu) { // Menu is closed
        document.getElementById("menu").style.width = menuWidth;
        document.getElementById("main").style.marginLeft = menuWidth;
        document.getElementById("main").style.width = "calc(100% - " + menuWidth + ")";
        document.getElementById("menu").style.borderRightWidth = "1px";
        document.getElementById("canvas").style.width = ('width', "calc(100% - 1000px)");
        
        console.log('Opening menu');
        openMenu = true;
    } else { // Menu is open
        document.getElementById("menu").style.width = "0px";
        document.getElementById("main").style.marginLeft = "0px";
        document.getElementById("menu").style.borderRightWidth = "0px";
        document.getElementById("main").style.width = "inherit";
        document.getElementById("canvas").style.width = "100%";
        console.log('Closing menu');
        openMenu = false;
    }
}

async function getCoordinates() {
    const parameter = encodeURIComponent(document.getElementById('location-field').value);
    const response = await fetch(apiURLBase + parameter + "&format=json&limit=1");
    const results = await response.json();
    const target = results[0];

    document.getElementById("longitude-field").value = target.lon;
    document.getElementById("latitude-field").value = target.lat;

    setMap(target.lat, target.lon);

    console.log(target);
}

function setMap(latitude, longitude) {
    map.setView([latitude, longitude], baseZoomLevel);
    // L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    //     maxZoom: 20,
    //     attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
    // }).addTo(map);

    var marker = L.marker([latitude, longitude]).addTo(map);
}