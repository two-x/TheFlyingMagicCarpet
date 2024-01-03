<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Subsystem Data Display</title>
  <style>
    table {
      border-collapse: collapse;
      width: 100%;
      margin-top: 20px;
    }

    th, td {
      border: 1px solid #dddddd;
      text-align: left;
      padding: 8px;
    }

    th {
      background-color: #f2f2f2;
    }

    select {
      margin-bottom: 10px;
    }
  </style>
</head>
<body>

  <label for="subsystemSelect">Select Subsystem:</label>
  <select id="subsystemSelect"></select>

  <table id="data-table">
    <thead>
      <tr>
        <th>Subsystem</th>
        <th>Name</th>
        <th>Value</th>
        <th>Units</th>
      </tr>
    </thead>
    <tbody></tbody>
  </table>

  <script>
    document.addEventListener('DOMContentLoaded', function () {
      // WebSocket connection
      const socket = new WebSocket('ws://your-websocket-server-url');

      // Subsystems
      const subsystems = ["Mode logic", "Diag", "Brake", "Gas", "Steering", "Temperature", "HotRC", "UI"];

      // Populate subsystem select options
      const subsystemSelect = document.getElementById('subsystemSelect');
      subsystems.forEach(subsystem => {
        const option = document.createElement('option');
        option.value = subsystem;
        option.text = subsystem;
        subsystemSelect.appendChild(option);
      });

      // Table body
      const tableBody = document.querySelector('#data-table tbody');

      // Handle WebSocket messages
      socket.addEventListener('message', function (event) {
        const data = JSON.parse(event.data);

        // Clear table before populating with new data
        tableBody.innerHTML = '';

        // Filter data based on selected subsystem
        const selectedSubsystem = subsystemSelect.value;
        const filteredData = data.filter(entry => entry.subsystem === selectedSubsystem);

        // Populate table with filtered data
        filteredData.forEach(entry => {
          const row = tableBody.insertRow();
          row.insertCell(0).textContent = entry.subsystem;
          row.insertCell(1).textContent = entry.name;
          row.insertCell(2).textContent = entry.value;
          row.insertCell(3).textContent = entry.units;
        });
      });

      // Initial data request
      socket.addEventListener('open', function () {
        // Send a request for initial data
        socket.send('getData');
      });

      // Change event listener for subsystem selection
      subsystemSelect.addEventListener('change', function () {
        // Send a request for data when the subsystem selection changes
        socket.send('getData');
      });
    });
  </script>

</body>
</html>