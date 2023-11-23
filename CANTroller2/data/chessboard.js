document.addEventListener("DOMContentLoaded", function() { setInterval(changeColor, randomInterval()); });
function changeColor() {
    const squares = document.querySelectorAll(".square");
    squares.forEach(square => { square.style.backgroundColor = getRandomColor(); });
}
function getRandomColor() {
    const componentToBeZero = Math.floor(Math.random() * 3);
    // Initialize RGB components
    let red = Math.floor(Math.random() * 256);
    let green = Math.floor(Math.random() * 256);
    let blue = Math.floor(Math.random() * 256);
    // Set the selected component to zero
    if (componentToBeZero === 0) { red = 0; }
    else if (componentToBeZero === 1) { green = 0; }
    else { blue = 0; }
    // Combine components to form the color in hex format
    const color = `#${red.toString(16).padStart(2, '0')}${green.toString(16).padStart(2, '0')}${blue.toString(16).padStart(2, '0')}`;
    return color;
    // return `rgb(${randomValue()}, ${randomValue()}, ${randomValue()})`;
}
function randomValue() {
    return Math.floor(Math.random() * 256);
}
function randomInterval() {
    return Math.floor(Math.random() * (300 - 100) + 200);
}