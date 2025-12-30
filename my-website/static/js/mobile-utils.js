// Mobile utilities for enhanced mobile experience
class MobileUtils {
  // Check if the device is mobile
  static isMobile() {
    return window.matchMedia('(max-width: 996px)').matches;
  }

  // Check if the device is a touch device
  static isTouchDevice() {
    return 'ontouchstart' in window || navigator.maxTouchPoints > 0;
  }

  // Check if the device is a tablet
  static isTablet() {
    return window.matchMedia('(min-width: 769px) and (max-width: 1024px)').matches;
  }

  // Add mobile-specific classes to body
  static addMobileClasses() {
    if (this.isMobile()) {
      document.body.classList.add('is-mobile');
    }
    if (this.isTablet()) {
      document.body.classList.add('is-tablet');
    }
    if (this.isTouchDevice()) {
      document.body.classList.add('is-touch');
    }
  }

  // Initialize mobile functionality
  static init() {
    // Add mobile classes on load
    this.addMobileClasses();

    // Update classes on resize
    window.addEventListener('resize', () => {
      // Remove all mobile classes
      document.body.classList.remove('is-mobile', 'is-tablet', 'is-touch');

      // Add appropriate classes
      this.addMobileClasses();
    });

    // Enhance touch interactions
    this.enhanceTouchInteractions();
  }

  // Enhance touch interactions
  static enhanceTouchInteractions() {
    // Add touch-friendly classes to interactive elements
    const interactiveElements = document.querySelectorAll(
      'a, button, input, textarea, select, [role="button"], [role="link"], [tabindex]'
    );

    interactiveElements.forEach(element => {
      // Add touch target enhancements
      element.classList.add('touch-target');
    });

    // Add touch event listeners for better mobile experience
    document.addEventListener('touchstart', function() {}, true);
  }

  // Optimize for mobile performance
  static optimizeForMobile() {
    if (this.isMobile()) {
      // Reduce animations on mobile for better performance
      const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
      if (prefersReducedMotion) {
        document.body.classList.add('reduce-motion');
      }
    }
  }

  // Handle mobile navigation
  static initMobileNavigation() {
    // Close mobile menu when clicking outside
    document.addEventListener('click', (e) => {
      const mobileMenu = document.querySelector('.mobile-navbar');
      const menuButton = document.querySelector('.menu-button');

      if (mobileMenu && !mobileMenu.contains(e.target) && !menuButton?.contains(e.target)) {
        // Close mobile menu if it's open
        const sidebar = document.querySelector('.sidebar');
        if (sidebar && sidebar.classList.contains('sidebar-open')) {
          sidebar.classList.remove('sidebar-open');
        }
      }
    });
  }
}

// Initialize mobile utilities when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
  MobileUtils.init();
  MobileUtils.optimizeForMobile();
  MobileUtils.initMobileNavigation();
});

// Re-initialize when window is resized
window.addEventListener('resize', () => {
  MobileUtils.addMobileClasses();
  MobileUtils.optimizeForMobile();
});