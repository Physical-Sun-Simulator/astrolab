var openMenu = false;
var menuWidth = "400px"

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