// Constants
const apiURLBase = "https://nominatim.openstreetmap.org/search?q=";
const baseLatitude = 51.4486602;
const baseLongitude = 5.4903995655080475;
const baseZoomLevel = 12;

// Global variables
var map = L.map('map').setView([baseLatitude, baseLongitude], baseZoomLevel);

// Initialization
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 20,
    attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
}).addTo(map);

/**
 * Provide the coordinates based on the given description.
 */
async function getCoordinates() {
    // Getting the data
    const parameter = encodeURIComponent(document.getElementById('location-field').value);
    const response = await fetch(apiURLBase + parameter + "&format=json&limit=1");
    const results = await response.json();
    const target = results[0];

    // Assigning the data
    document.getElementById("longitude-field").value = target.lon;
    document.getElementById("latitude-field").value = target.lat;

    // Indicating the data
    map.setView([target.lat, target.lon], baseZoomLevel);
    var marker = L.marker([target.lat, target.lon]).addTo(map);
}