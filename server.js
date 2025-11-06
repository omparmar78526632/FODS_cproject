const express = require('express');
const { execFile } = require('child_process'); // To run C program
const cors = require('cors'); // To allow frontend to connect
const path = require('path');

const app = express();
const port = 3000;

// --- Middleware ---
app.use(cors()); // Allow cross-origin requests
app.use(express.json()); // Parse JSON request bodies
app.use(express.static('public')); // Serve frontend files

// --- API Endpoint ---
app.post('/calculate_route', (req, res) => {
  console.log('Received request:', req.body);
  const { source, destination, algorithm } = req.body;

  // 1. Get the path to the C executable
  //    (On Windows the compiled executable will be 'routing_app.exe')
  const programPath = path.join(__dirname, process.platform === 'win32' ? 'routing_app.exe' : 'routing_app');

  // 2. Define the arguments for the C program
  const args = [source, destination, algorithm];

  console.log(`Running: ${programPath} ${args.join(' ')}`);

  // 3. Execute the C file
  execFile(programPath, args, (error, stdout, stderr) => {
    if (error) {
      console.error(`exec error: ${error}`);
      return res.status(500).json({ success: false, error: error.message });
    }
    if (stderr) {
      console.error(`stderr: ${stderr}`);
      return res.status(500).json({ success: false, error: stderr });
    }

    // 4. Success! Parse the JSON output from C and send to frontend
    try {
      console.log('C Program Output:', stdout);
      const results = JSON.parse(stdout);
      res.json(results);
    } catch (parseError) {
      console.error('JSON Parse Error:', parseError);
      console.error('Raw C Output:', stdout);
      res.status(500).json({ success: false, error: 'Failed to parse C program output.' });
    }
  });
});

app.listen(port, () => {
  console.log(`Routing server listening at http://localhost:${port}`);
});