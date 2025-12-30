@echo off
REM Test responsive design by building the site
echo Testing responsive design implementation...
npm run build
if %errorlevel% neq 0 (
    echo Build failed. Please check for errors.
    exit /b 1
) else (
    echo Build successful. Responsive design is ready for testing.
    echo To test locally, run: npm run serve
)