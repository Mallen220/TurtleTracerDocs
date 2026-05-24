// @ts-check
import { defineConfig } from "astro/config";
import starlight from "@astrojs/starlight";

// https://astro.build/config
export default defineConfig({
  site: "https://www.turtletracer.com/",
  integrations: [
    starlight({
      title: "Turtle Tracer Docs",
      logo: {
        src: "./public/icon.png",
      },
      favicon: "/favicon.png",
      lastUpdated: true,
      customCss: ["./src/styles/custom.css"],
      components: {
        Head: "./src/components/Head.astro",
        SocialIcons: "./src/components/CustomSocialIcons.astro",
        PageTitle: "./src/components/PageTitle.astro",
        Footer: "./src/components/CustomFooter.astro",
      },
      sidebar: [
        // {
        // 	label: 'Guides',
        // 	items: [
        // 		// Each item here is one entry in the navigation menu.
        // 		{ label: 'Example Guide', slug: 'guides/example' },
        // 	],
        // },
        {
          label: "TurtleTracerLib",
          items: [
            { label: "Installation", slug: "turtle-tracer-lib/installation" },
            {
              label: "Getting Started",
              slug: "turtle-tracer-lib/getting-started",
            },
            { label: "Live View", slug: "turtle-tracer-lib/live-view" },
            {
              label: "Progress Tracker",
              slug: "turtle-tracer-lib/progress-tracker",
            },
            {
              label: "Named Commands & Events",
              slug: "turtle-tracer-lib/named-commands",
            },
            {
              label: "TurtleTracerReader",
              slug: "turtle-tracer-lib/turtle-tracer-reader",
            },
          ],
        },
        {
          label: "TurtleTracer",
          items: [
            { label: "Installation", slug: "turtle-tracer/installation" },
            { label: "Browser Support", slug: "turtle-tracer/browser" },
            { label: "Getting Started", slug: "turtle-tracer/getting-started" },
            { label: "File Management", slug: "turtle-tracer/file-management" },
            { label: "Path Editing", slug: "turtle-tracer/path-editing" },
            { label: "Heading Types", slug: "turtle-tracer/heading-types" },
            {
              label: "Simulation & Playback",
              slug: "turtle-tracer/simulation",
            },
            { label: "Telemetry Overlay", slug: "turtle-tracer/telemetry" },
            { label: "Obstacles", slug: "turtle-tracer/obstacles" },
            { label: "Event Markers", slug: "turtle-tracer/event-markers" },
            { label: "Path Validation", slug: "turtle-tracer/validation" },
            { label: "Path Optimization", slug: "turtle-tracer/optimization" },
            { label: "Exporting Code", slug: "turtle-tracer/exporting" },
            { label: "Plugins", slug: "turtle-tracer/plugins" },
            { label: "Settings", slug: "turtle-tracer/settings" },
            { label: "Controls & Shortcuts", slug: "turtle-tracer/controls" },
            { label: "Left Sidebar", slug: "turtle-tracer/sidebar" },
            {
              label: "Troubleshooting & FAQ",
              slug: "turtle-tracer/troubleshooting",
            },
          ],
        },
        // ,
        // {
        // 	label: 'Reference',
        // 	autogenerate: { directory: 'reference' },
        // },
      ],
    }),
  ],
});
