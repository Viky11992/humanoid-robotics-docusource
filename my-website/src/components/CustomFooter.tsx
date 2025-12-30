import React from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import { FooterLinkItem, useFooterLinks } from '@theme/Footer/FooterLinkItem';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';

function FooterLink({
  item,
  className
}) {
  const { to, href, label, prependBaseUrlToHref, ...props } = item;
  const toUrl = useBaseUrl(to);
  const normalizedHref = useBaseUrl(href, { forcePrependBaseUrl: true });
  return (
    <Link
      className={clsx('footer__link-item', className)}
      {...props}
      href={href ? prependBaseUrlToHref ? normalizedHref : href : undefined}
      to={toUrl}
      target={href ? '_blank' : undefined}
      rel={href ? 'noopener noreferrer' : undefined}>
      {label}
    </Link>
  );
}

function SocialFooter() {
  const links = [
    {
      title: 'GitHub',
      href: 'https://github.com/Viky11992',
      className: 'github-link'
    },
    {
      title: 'LinkedIn',
      href: 'https://www.linkedin.com/in/shoaibarshad92/',
      className: 'linkedin-link'
    },
    {
      title: 'Twitter/X',
      href: 'https://x.com/ShoaibVickey',
      className: 'twitter-link'
    },
    {
      title: 'Email',
      href: 'mailto:Shoaibarshad470@gmail.com',
      className: 'email-link'
    },
    {
      title: 'Facebook',
      href: 'https://www.facebook.com/profile.php?id=100012378756441',
      className: 'facebook-link'
    }
  ];

  return (
    <div className="footer__social-section">
      <div className="footer__title">Connect With Me</div>
      <div className="footer__social-links">
        {links.map((link, index) => (
          <FooterLink key={index} item={link} className={link.className} />
        ))}
      </div>
    </div>
  );
}

export default function Footer() {
  const location = useLocation();
  const links = useFooterLinks();

  // Don't show footer on 404 pages
  if (location.pathname === '/404') {
    return null;
  }

  return (
    <footer className="footer">
      <div className="container container-fluid">
        <div className="footer__social">
          <SocialFooter />
        </div>
        <div className="footer__bottom">
          <div className="footer__links">
            {links.map((linkItem, i) => (
              <div key={i} className="footer__col">
                <div className="footer__title">{linkItem.title}</div>
                <ul className="footer__items">
                  {linkItem.items.map((item, j) => (
                    <li key={j} className="footer__item">
                      <FooterLinkItem item={item} />
                    </li>
                  ))}
                </ul>
              </div>
            ))}
          </div>
          <div className="footer__copyright">
            <div className="footer__copyright-text">
              Copyright © {new Date().getFullYear()} My Project, Inc. Built with Docusaurus.
            </div>
          </div>
        </div>
      </div>
    </footer>
  );
}