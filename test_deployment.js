// Test script to verify Vercel/Railway deployment configuration
const https = require('https');
const http = require('http');
const url = require('url');

async function testAPIConnection(backendUrl) {
  return new Promise((resolve, reject) => {
    const parsedUrl = new URL(backendUrl);
    const client = parsedUrl.protocol === 'https:' ? https : http;

    const options = {
      hostname: parsedUrl.hostname,
      port: parsedUrl.port || (parsedUrl.protocol === 'https:' ? 443 : 80),
      path: parsedUrl.pathname + '/api/v1/health',
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
        'User-Agent': 'Vercel-Railway-Test-Client'
      }
    };

    const req = client.request(options, (res) => {
      let data = '';

      res.on('data', (chunk) => {
        data += chunk;
      });

      res.on('end', () => {
        try {
          const response = JSON.parse(data);
          resolve({
            success: true,
            statusCode: res.statusCode,
            response: response
          });
        } catch (e) {
          resolve({
            success: false,
            statusCode: res.statusCode,
            error: 'Invalid JSON response',
            raw: data
          });
        }
      });
    });

    req.on('error', (error) => {
      resolve({
        success: false,
        error: error.message
      });
    });

    req.setTimeout(10000, () => {
      req.destroy();
      resolve({
        success: false,
        error: 'Request timeout after 10 seconds'
      });
    });

    req.end();
  });
}

async function runTests() {
  console.log('Testing Vercel/Railway deployment configuration...\n');

  // Test URLs for different environments
  const testUrls = [
    'https://your-railway-backend-production.up.railway.app',
    'https://humanoid-robotics.vercel.app',
    'http://127.0.0.1:8000'
  ];

  for (const testUrl of testUrls) {
    console.log(`Testing: ${testUrl}`);
    try {
      const result = await testAPIConnection(testUrl);
      if (result.success) {
        console.log(`  ✓ Success - Status: ${result.statusCode}`);
        console.log(`  Response:`, result.response);
      } else {
        console.log(`  ✗ Failed - Error: ${result.error}`);
        if (result.statusCode) {
          console.log(`  Status Code: ${result.statusCode}`);
        }
      }
    } catch (error) {
      console.log(`  ✗ Error during test: ${error.message}`);
    }
    console.log('');
  }
}

// Run the tests
runTests().catch(console.error);