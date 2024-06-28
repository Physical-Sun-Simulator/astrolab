// Configuration constants
const CONFIGURATION_DELAY = 500;
const CONFIGURATION_PATH = "/configuration";
const configurationInterval = setInterval(
    updateConfiguration,
    CONFIGURATION_DELAY,
    CONFIGURATION_PATH
)

// Open menu constants
const OPEN_MENU_WIDTH = "400px"
const OPEN_MAIN_WIDTH = "calc(100% - " + OPEN_MENU_WIDTH + ")";
const OPEN_BORDER_WIDTH = "1px";
const OPEN_CANVAS_WIDTH = ('width', "calc(100% - 1000px)");

// Closed menu constants
const CLOSED_MENU_WIDTH = "0px";
const CLOSED_BORDER_WIDTH = "0px";
const CLOSED_MAIN_WIDTH = "inherit";
const CLOSED_CANVAS_WIDTH = "100%";

// Menu variables
var openMenu = false;

/**
 * Change menu to the opposite state.
 */
function changeMenu() {
    if (!openMenu) { // Menu is closed
        // Open menu
        document.getElementById("menu").style.width = OPEN_MENU_WIDTH;
        document.getElementById("main").style.marginLeft = OPEN_MENU_WIDTH;
        document.getElementById("main").style.width = OPEN_MAIN_WIDTH;
        document.getElementById("menu").style.borderRightWidth = OPEN_BORDER_WIDTH;
        document.getElementById("canvas").style.width = OPEN_CANVAS_WIDTH;
        openMenu = true;
    } else { // Menu is open
        // Close menu
        document.getElementById("menu").style.width = CLOSED_MENU_WIDTH;
        document.getElementById("main").style.marginLeft = CLOSED_MENU_WIDTH;
        document.getElementById("menu").style.borderRightWidth = CLOSED_BORDER_WIDTH;
        document.getElementById("main").style.width = CLOSED_MAIN_WIDTH;
        document.getElementById("canvas").style.width = CLOSED_CANVAS_WIDTH;
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
            document.getElementById("elevation-position-field").value = Math.round(json.elevation);
            document.getElementById("azimuth-position-field").value = Math.round(json.azimuth);
        });
}
