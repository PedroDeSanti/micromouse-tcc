// @ts-check
import { defineConfig } from "astro/config";
import starlight from "@astrojs/starlight";

// https://astro.build/config
export default defineConfig({
  site: "https://tcc.desanti.dev/",
  integrations: [
    starlight({
      title: "Micras",
      social: [
        {
          icon: "github",
          label: "GitHub",
          href: "https://github.com/team-micras",
        },
      ],
      sidebar: [
        // {
        //   label: "Guides",
        //   items: [
        //     // Each item here is one entry in the navigation menu.
        //     { label: "Example Guide", slug: "guides/example" },

        //   ],
        // },
        // {
        //   label: "Reference",
        //   autogenerate: { directory: "reference" },
        // },

        { label: "Computação", slug: "computação" },
        { label: "Elétrica", slug: "elétrica" },
        { label: "Mecânica", slug: "mecânica" },
      ],
    }),
  ],
});
