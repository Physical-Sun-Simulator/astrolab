// Constants
const MENU_WIDTH = "400px"
const CONFIGURATION_DELAY = 500;
const CONFIGURATION_PATH = "/configuration";

// Global variables
var openMenu = false;

// Read json updates info into configuration
const configurationInterval = setInterval(updateConfiguration, CONFIGURATION_DELAY, CONFIGURATION_PATH)

/**
 * Change menu to the opposite state.
 */
function changeMenu() {
    if (!openMenu) { // Menu is closed
        // Open menu
        document.getElementById("menu").style.width = MENU_WIDTH;
        document.getElementById("main").style.marginLeft = MENU_WIDTH;
        document.getElementById("main").style.width = "calc(100% - " + MENU_WIDTH + ")";
        document.getElementById("menu").style.borderRightWidth = "1px";
        document.getElementById("canvas").style.width = ('width', "calc(100% - 1000px)");
        openMenu = true;
    } else { // Menu is open
        // Close menu
        document.getElementById("menu").style.width = "0px";
        document.getElementById("main").style.marginLeft = "0px";
        document.getElementById("menu").style.borderRightWidth = "0px";
        document.getElementById("main").style.width = "inherit";
        document.getElementById("canvas").style.width = "100%";
        openMenu = false;
    }
}

/**
 * Update configuration from json file.
 */
function updateConfiguration(path) {
    // Get configuration json
    fetch(path)
    .then((response) => response.json())
    .then((json) => {
        // Assign json to configuration fields
        document.getElementById("elevation-position-field").value = json.elevation;
        document.getElementById("azimuth-position-field").value = json.azimuth;
    });
}
